#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "dram_controller/impl/plugin/prac/prac.h"
#include "dram_controller/impl/plugin/device_config/device_config.h"

#include <limits>
#include <vector>
#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace Ramulator {

class PRAC : public IControllerPlugin, public Implementation, public IPRAC {
    RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, PRAC, "PRAC", "PRAC.")

private:
    class PerBankCounters;

private:
    DeviceConfig m_cfg;
    std::vector<PRAC::PerBankCounters> m_bank_counters;
    std::vector<int> m_same_bank_offsets;

    Clk_t m_clk = 0;

    ABOState m_state = ABOState::NORMAL;
    Clk_t m_abo_recovery_start = std::numeric_limits<Clk_t>::max();

    int m_abo_act_ns = -1;
    int extension = 0;
    int m_abo_recovery_refs = -1;
    int m_abo_delay_acts = -1;
    int m_abo_thresh = -1;
    bool trefi_reset = false; //reset 기능 
    int m_abo_act_cycles = -1;
    int m_refresh_count = 0; // Refresh request 발생 횟수
    uint32_t m_abo_recov_rem_refs = -1;
    uint32_t m_abo_delay_rem_acts = -1;
    bool m_is_abo_needed = false;
    bool extend_victim = param<bool>("Extend").default_val(false); // 기본값은 false
    bool adaptive_prac = false;
    bool serviceQ = false;
    bool m_debug = false;

    uint64_t s_num_recovery = 0;

    int m_refresh_req_id = -1; // Refresh request ID 추가

    // 새로 추가된 멤버 변수
    Clk_t m_last_refresh_clk = -1; // 마지막 refresh request 처리 시점
    bool m_act_received_since_refresh = true; // Refresh 이후 ACT request 수신 여부

public:
    void init() override { 
        m_debug = param<bool>("debug").default_val(false);
        m_abo_delay_acts = param<int>("abo_delay_acts").default_val(4);
        m_abo_recovery_refs = param<int>("abo_recovery_refs").default_val(4);
        m_abo_act_ns = param<int>("abo_act_ns").default_val(180);
        m_abo_thresh = param<int>("abo_threshold").default_val(512);
        trefi_reset = param<bool>("trefi_reset").default_val(false); // YAML에서 trefi_reset 값 읽기
        adaptive_prac = param<bool>("adaptive_prac").default_val(false); // YAML에서 adaptive_prac 값 읽기   
        serviceQ = param<bool>("serviceQ").default_val(false); // YAML에서 adaptive_prac 값 읽기   
    }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
        m_cfg.set_device(cast_parent<IDRAMController>());
        init_dram_params(m_cfg.m_dram);

        m_is_abo_needed = false;
        m_abo_act_cycles = m_abo_act_ns / ((float) m_cfg.m_dram->m_timing_vals("tCK_ps") / 1000.0f);
        // YAML 파일에서 설정값 읽기 (없으면 기본값 사용)
        int add_thres = param<int>("add_thres").default_val(32); // 기본값 32
        int max_thres_rows = param<int>("max_thres_rows").default_val(8); // 기본값 8

        m_bank_counters.reserve(m_cfg.m_num_banks);
        for (int i = 0; i < m_cfg.m_num_banks; i++) {
            m_bank_counters.emplace_back(i, m_cfg, m_is_abo_needed, m_abo_thresh, m_debug, add_thres, max_thres_rows, adaptive_prac, serviceQ);
        }

        // Refresh request ID 초기화
        m_refresh_req_id = m_cfg.m_dram->m_requests("all-bank-refresh");

        register_stat(s_num_recovery).name("prac_num_recovery");
    }

    void update(bool request_found, ReqBuffer::iterator& req_it) override {
        m_clk++;
        auto& req = *req_it;
        update_state_machine(request_found, *req_it);

        if (!request_found) {
            return;
        }

        

        // Refresh request 처리

        auto& req_meta = m_cfg.m_dram->m_command_meta(req.command);
        auto& req_scope = m_cfg.m_dram->m_command_scopes(req.command);

        // ACT request 처리
        if (req.command == m_cfg.m_dram->m_commands("ACT")) {
            m_act_received_since_refresh = true; // ACT request 수신 플래그 설정
        }

        bool has_bank_wildcard = req.addr_vec[m_cfg.m_bank_level] == -1;
        bool has_bankgroup_wildcard = req.addr_vec[m_cfg.m_bankgroup_level] == -1;
        if (has_bankgroup_wildcard && has_bank_wildcard) { // All BG, All Bank
            int offset = req.addr_vec[m_cfg.m_rank_level] * m_cfg.m_num_banks_per_rank;
            for (int i = 0; i < m_cfg.m_num_banks_per_rank; i++) {
                m_bank_counters[offset + i].on_request(req);
            }
            req.addr_vec[m_cfg.m_bank_level] = -1;
        }
        else if (has_bankgroup_wildcard) { // All BG, Single Bank
            int rank_offset = req.addr_vec[m_cfg.m_rank_level] * m_cfg.m_num_banks_per_rank;
            int bank_offset = req.addr_vec[m_cfg.m_bank_level];
            for (int i = 0; i < m_cfg.m_num_bankgroups; i++) {
                int bg_offset = i * m_cfg.m_num_banks_per_bankgroup;
                m_bank_counters[rank_offset + bg_offset + bank_offset].on_request(req);
            }
        }
        else if (has_bank_wildcard) { // Single BG, All Bank
            int rank_offset = req.addr_vec[m_cfg.m_rank_level] * m_cfg.m_num_banks_per_rank;
            int bg_offset = req.addr_vec[m_cfg.m_bankgroup_level] * m_cfg.m_num_banks_per_bankgroup; 
            for (int i = 0; i < m_cfg.m_num_banks_per_bankgroup; i++) {
                m_bank_counters[rank_offset + bg_offset + i].on_request(req);
            }
        }
        else { // Single BG, Single Bank
            auto flat_bank_id = m_cfg.get_flat_bank_id(req);
            m_bank_counters[flat_bank_id].on_request(req);
        }
    }

    void update_state_machine(bool request_found, const Request& req) {
        std::unordered_map<ABOState, std::string> state_names = {
            {ABOState::NORMAL, "ABOState::NORMAL"},
            {ABOState::PRE_RECOVERY, "ABOState::PRE_RECOVERY"},
            {ABOState::RECOVERY, "ABOState::RECOVERY"},
            {ABOState::DELAY, "ABOState::DELAY"}
        };
        auto cmd_prea = m_cfg.m_dram->m_commands("PREA");
        auto cmd_rfmab = m_cfg.m_dram->m_commands("RFMab");
        auto cmd_rfmsb = m_cfg.m_dram->m_commands("RFMsb");
        auto cmd_act = m_cfg.m_dram->m_commands("ACT");
        auto cur_state = m_state;
        
        switch(m_state) {
        case ABOState::NORMAL:
            
            if (m_is_abo_needed) {
                if (m_debug) {
                    std::printf("[PRAC] [%lu] <%s> Asserting ALERT_N.\n", m_clk, state_names[cur_state].c_str());
                }
                m_state = ABOState::PRE_RECOVERY;
                m_abo_recovery_start = m_clk + m_abo_act_cycles;
                s_num_recovery++;
                
            }
            if (req.command == m_refresh_req_id) {
            
                // 연속된 refresh request 무시
                if (m_last_refresh_clk != -1 && !m_act_received_since_refresh) {
                    //std::cout << "[PRAC] Ignoring consecutive refresh request at clock " << m_clk << std::endl;
                    return;
                }
                //std::cout << "[PRAC] Critical rows for all banks:" << std::endl;
                
                //for (const auto& counter : m_bank_counters) {
                  //  std::cout << "[PRAC] [Bank " << counter.get_bank_id() << "] Critical Rows:" << std::endl;
                  //  for (const auto& row : counter.get_critical_rows()) {
                  //      std::cout << "[Unreal] [" << counter.get_bank_id() << "] Row: " << row.first << ", Count: " << row.second << std::endl;
                //    }
                //}
                
                handle_refresh_request(req);
                m_last_refresh_clk = m_clk;
                m_act_received_since_refresh = false; // Refresh 이후 ACT request 수신 여부 초기화
                return;
            }
            break;
        case ABOState::PRE_RECOVERY:
            if (request_found && req.command == cmd_prea) {
                if (m_debug) {
                    //std::printf("[PRAC] [%lu] <%s> Received PREA.\n", m_clk, state_names[cur_state].c_str());
                }
            }
            if (m_clk == m_abo_recovery_start) {
                m_state = ABOState::RECOVERY;
                m_abo_recovery_start = std::numeric_limits<Clk_t>::max();
                m_abo_recov_rem_refs = m_abo_recovery_refs * m_cfg.m_num_ranks;
            }
            break;
        case ABOState::RECOVERY:
            if (request_found && (req.command == cmd_rfmab ||
                req.command == cmd_rfmsb)) {
                m_abo_recov_rem_refs--;
                if (!m_abo_recov_rem_refs) {
                    m_state = ABOState::DELAY;
                    m_abo_delay_rem_acts = m_abo_delay_acts;
                }
            }
            break;
        case ABOState::DELAY:
            if (request_found && req.command == cmd_act) {
                m_abo_delay_rem_acts--;
                if (!m_abo_delay_rem_acts) {
                    m_is_abo_needed = false;
                    for (int i = 0; i < m_cfg.m_num_banks; i++) {
                        m_is_abo_needed |= m_bank_counters[i].is_critical();
                    }
                    m_state = ABOState::NORMAL;
                }
            }
            break;
        }
        if (m_debug && cur_state != m_state) {
            //std::printf("[PRAC] [%lu] <%s> -> <%s>\n", m_clk, state_names[cur_state].c_str(), state_names[m_state].c_str());
        }
    }

    Clk_t next_recovery_cycle() override {
        return m_abo_recovery_start;
    }

    int get_num_abo_recovery_refs() override {
        return m_abo_recovery_refs;
    }

    ABOState get_state() override {
        return m_state;
    }

private:
/*
void handle_refresh_request(const Request& req) {
    // Refresh request 처리 로직
    std::cout << "[PRAC] Refresh request received at clock " << m_clk << std::endl;
    if (trefi_reset) { // trefi_reset이 true일 경우에만 실행
    for (auto& counter : m_bank_counters) {
        auto& counters = counter.get_counters();
        auto& recently_accessed_rows = counter.get_recently_accessed_rows();

        for (auto it = counters.begin(); it != counters.end(); ++it) {
            // count가 0이 아니고, recently_accessed_rows에 없는 경우만 초기화
            if (it->second != 0 && recently_accessed_rows.find(it->first) == recently_accessed_rows.end()) {
                if (m_debug) {
                    std::printf("[PRAC] [%d] Resetting Count for Row: %d. Previous Count: %u\n", 
                                counter.get_bank_id(), it->first, it->second);
                }
                it->second = 0; // count 초기화
                if (m_debug) {
                    std::printf("[PRAC] [%d] Count for Row: %d after Reset: %u\n", 
                                counter.get_bank_id(), it->first, it->second);
                }
            }
        }
        // m_recently_accessed_rows 초기화
        recently_accessed_rows.clear();
    }
    }   
    
    if (m_clk % 3 == 0) {
        std::cout << "[PRAC] Refresh request is a multiple of 3. Removing oldest rows from m_thres_rows." << std::endl;
        for (auto& counter : m_bank_counters) {
            if (!counter.get_thres_rows().empty()) {
                // 가장 오래된 row 제거
                auto oldest = counter.get_thres_rows().begin();
                std::cout << "[PRAC] [Bank " << counter.get_bank_id() << "] Removing Row: " << oldest->first << ", Count: " << oldest->second << std::endl;
                counter.remove_oldest_thres_row();
            }
        }
    }
        
for (auto& counter : m_bank_counters) {
    //std::cout << "[PRAC] [Bank " << counter.get_bank_id() << "] Threshold Rows:" << std::endl;
    for (const auto& row : counter.get_thres_rows()) {
        std::cout << "[Service Q] [" << counter.get_bank_id() << "] Row: " << row.first << ", Count: " << row.second << std::endl;
    }
        //counter.reset(); // 예: 모든 bank counter 초기화
    }
}
*/

void handle_refresh_request(const Request& req) {
    // Refresh request 처리 로직
    
    //std::cout << "[PRAC] Refresh request received at clock " << m_clk << std::endl;
    m_refresh_count++;
    
    std::printf("[Internel-ref] count : %d\n", m_refresh_count);
    for (auto& counter : m_bank_counters) {
        for (const auto& row : counter.get_thres_rows()) {
            std::cout << "[Service Q list] [" << counter.get_bank_id() << "] Row: " << row.first << ", Count: " << row.second << std::endl;  
        }     
    }
    for (auto& counter : m_bank_counters) {
        auto& counters = counter.get_counters();
        auto& thres_rows = counter.get_thres_rows(); // 수정 가능한 참조
        auto& recently_accessed_rows = counter.get_recently_accessed_rows();
        
        // m_refresh_count가 3의 배수일 때만 실행
        if (m_refresh_count % 3 == 0) {
            if (counter.get_bank_id() == 0) {
            if (extend_victim==true){
            if (extension == 0) {
                extension = 1;
            }
            else{
                extension = 0;
            }
            }
            }
            if (extension == 0) {
                auto max_it = std::max_element(
                    thres_rows.begin(), thres_rows.end(),
                    [](const std::pair<int, uint32_t>& a, const std::pair<int, uint32_t>& b) {
                        return a.second < b.second;
                    });

                if (max_it != thres_rows.end()) {
                    int row_addr = max_it->first; // count가 가장 높은 row의 주소
                    uint32_t max_count = max_it->second;

                    // m_thres_rows에서 해당 row 제거
                    thres_rows.erase(max_it);
                    auto act_max = counters.find(row_addr); 
                    // m_counters에서 해당 row의 count 초기화
                    if (act_max != counters.end()) {
                        //std::printf("[PRAC]");
                        std::printf("[Service Q remove] [%d] Row: %d from m_thres_rows with Count: %u and reset its COUNT in m_counters.\n",
                            counter.get_bank_id(), act_max->first, counters[act_max->first]);
                        counters[act_max->first] = 0;
                        //std::printf("[After] [Bank %d] Removed Row: %d from m_thres_rows with Count: %u and reset its COUNT in m_counters.\n",
                         //   counter.get_bank_id(), act_max->first, counters[act_max->first]);
                        //counters[act_max->first] = 0;

                        //if (m_debug) {
                           // std::printf("[PRAC] [Bank %d] Removed Row: %d from m_thres_rows with Count: %u and reset its COUNT in m_counters.\n",
                             //           counter.get_bank_id(), act_max->first, max_count);
                        //}
                    }
                } 
            }       
            else {
                // 가장 큰 값을 가진 row를 찾음
            auto max_it = std::max_element(
             thres_rows.begin(), thres_rows.end(),
              [](const std::pair<int, uint32_t>& a, const std::pair<int, uint32_t>& b) {
              return a.second < b.second;
                });

            if (max_it != thres_rows.end()) {
                int row_addr = max_it->first; // count가 가장 높은 row의 주소
                 uint32_t max_count = max_it->second;

    // m_thres_rows에서 해당 row 제거
                    thres_rows.erase(max_it);

              // m_counters에서 해당 row의 count 초기화
               auto act_max = counters.find(row_addr);
              if (act_max != counters.end()) {
                 counters[act_max->first] = 0;
                        std::printf("[Service Q remove] [%d] Row: %d from m_thres_rows with Count: %u and reset its COUNT in m_counters.\n",
                             counter.get_bank_id(), act_max->first, max_count);
                 
             }

    // 두 번째로 큰 값을 가진 row를 찾음
                auto second_max_it = std::max_element(
                 thres_rows.begin(), thres_rows.end(),
                  [](const std::pair<int, uint32_t>& a, const std::pair<int, uint32_t>& b) {
                      return a.second < b.second;
                    });

                  if (second_max_it != thres_rows.end()) {
                   int second_row_addr = second_max_it->first; // 두 번째로 높은 row의 주소
                    uint32_t second_max_count = second_max_it->second;

              // m_thres_rows에서 해당 row 제거
                    thres_rows.erase(second_max_it);

            // m_counters에서 해당 row의 count 초기화
              auto second_act_max = counters.find(second_row_addr);
                    if (second_act_max != counters.end()) {
                    counters[second_act_max->first] = 0;

                
                       std::printf("[Service Q remove] [%d] Row: %d from m_thres_rows with Count: %u and reset its COUNT in m_counters.\n",
                            counter.get_bank_id(), second_act_max->first, second_max_count);
            
        }
        
    } 
      
 
            }
            
        }
        
        }
        if (trefi_reset) {
            for (auto it = counters.begin(); it != counters.end(); ++it) {
                // count가 0이 아니고, recently_accessed_rows에 없는 경우만 초기화
                if (it->second != 0 && recently_accessed_rows.find(it->first) == recently_accessed_rows.end()) {
                    if (m_debug) {
                        std::printf("[Reset] [%d] Resetting Count for Row: %d. Previous Count: %u\n", 
                                    counter.get_bank_id(), it->first, it->second);
                    }
                    it->second = 0; // count 초기화
                    if (m_debug) {
                        //std::printf("[Reset] [%d] Count for Row: %d after Reset: %u\n", 
                                    counter.get_bank_id(), it->first, it->second);
                    }
                }
            }

            // 기존 로직: recently_accessed_rows 초기화
            recently_accessed_rows.clear();
        }
    }
    
    // 각 bank의 m_thres_rows 출력

}

class PerBankCounters {
    public: 
    const std::unordered_map<int, uint32_t>& get_counters() const {
        return m_counters;
    }

    std::unordered_map<int, uint32_t>& get_counters() {
        return m_counters;
    }

    const std::unordered_set<int>& get_recently_accessed_rows() const {
        return m_recently_accessed_rows;
    }

    std::unordered_set<int>& get_recently_accessed_rows() {
        return m_recently_accessed_rows;
    }
        int get_bank_id() const {
            return m_bank_id;
        }

        const std::unordered_map<int, uint32_t>& get_critical_rows() const {
            return m_critical_rows;
        }

        std::unordered_map<int, uint32_t>& get_thres_rows() {
            return m_thres_rows;
        }

    PerBankCounters(int bank_id, DeviceConfig& cfg, bool& is_abo_needed, int alert_thresh, bool debug, int add_thres = -1, int max_thres_rows = 4, bool adaptive_prac = false, bool serviceQ = false)
    : m_bank_id(bank_id), m_cfg(cfg), m_is_abo_needed(is_abo_needed),
      m_alert_thresh(alert_thresh), m_add_thres(add_thres), m_max_thres_rows(max_thres_rows), m_debug(debug), m_adaptive_prac(adaptive_prac), m_serviceQ(serviceQ)   {
    init_dram_params(m_cfg.m_dram);
    reset();
        }

        ~PerBankCounters() {
            m_counters.clear();    
            m_thres_rows.clear();
        }

        void on_request(const Request& req) {
            if (m_handlertable.find(req.command) != m_handlertable.end()) {
                m_handlertable[req.command].handler(req);
            }
        }

        void init_dram_params(IDRAM* dram) {
            CommandHandler handlers[] = {
                // TODO: We should process PREs? Doesn't really change the results though.
                {std::string("ACT"), std::bind(&PerBankCounters::process_act, this, std::placeholders::_1)},
                {std::string("RFMab"), std::bind(&PerBankCounters::process_rfm, this, std::placeholders::_1)},
                {std::string("RFMsb"), std::bind(&PerBankCounters::process_rfm, this, std::placeholders::_1)}
            };
            for (auto& h : handlers) {
                if (!dram->m_commands.contains(h.cmd_name)) {
                    std::cout << "[PRAC] Command " << h.cmd_name << "does not exist." << std::endl;
                    exit(0);
                }
                m_handlertable[dram->m_commands(h.cmd_name)] = h;
            }
        }
        void remove_oldest_thres_row() {
            if (!m_thres_rows.empty()) {
                auto oldest = m_thres_rows.begin(); // 가장 오래된 항목
                int row_addr = oldest->first;
        
                // m_thres_rows에서 제거
                m_thres_rows.erase(oldest);
        
                // m_counters에서 COUNT 값을 reset
                if (m_counters.find(row_addr) != m_counters.end()) {
                    m_counters[row_addr] = 0;
                    if (m_debug) {
                        std::printf("[PRAC] [%d] Reset COUNT for Row: %d\n", m_bank_id, row_addr);
                    }
                }
            }
        }
        void reset() {
            m_counters.clear();
            m_critical_rows.clear();
        m_thres_rows.clear();
        }

        bool is_critical() {
            return m_critical_rows.size() > 0;
        }

    private:
    std::unordered_set<int> m_recently_accessed_rows; // 최근 ACT로 접근한 row를 추적
        struct CommandHandler {
            std::string cmd_name;
            std::function<void(const Request&)> handler;
        };

        DeviceConfig& m_cfg;
        bool& m_is_abo_needed;
        bool m_adaptive_prac = false; // 기본값은 false
        bool m_serviceQ = false; // 기본값은 false
        std::unordered_map<int, uint32_t> m_counters;
        std::unordered_map<int, uint32_t> m_critical_rows;
    std::unordered_map<int, uint32_t> m_thres_rows; // 새로운 m_thres_rows 추가
        std::unordered_map<int, CommandHandler> m_handlertable;

        int m_alert_thresh = -1;
    int m_add_thres = -1; // 새로운 임계값 추가
    int m_adapt_thres = -1; // adaptive_prac 임계값 추가
    int m_max_thres_rows = 4; // m_thres_rows의 최대 크기
    int m_adaptive_thres_rows ; // m_thres_rows의 최대 크기
        bool m_debug = false;
        int m_bank_id = -1;
/* 정상 동작 (add_thres_row가 제한 없음)
        void process_act(const Request& req) {
            auto row_addr = req.addr_vec[m_cfg.m_row_level];    
            if (m_counters.find(row_addr) == m_counters.end()) {
                m_counters[row_addr] = 0;
            }
            m_counters[row_addr]++;
            //if (m_debug) {
             //   std::printf("[PRAC] [%d] [ACT] Row: %d Act: %u\n",
             //       m_bank_id, row_addr, m_counters[row_addr]);
            //}
            if (m_counters[row_addr] >= m_alert_thresh) {
                m_critical_rows[row_addr] = m_counters[row_addr];
                m_is_abo_needed = true;
            }

        // 새로운 m_thres_rows 처리
        if (m_add_thres != -1 && m_counters[row_addr] >= m_add_thres) {
            m_thres_rows[row_addr] = m_counters[row_addr];
            if (m_debug) {
            //    std::printf("[PRAC] [%d] [THRES] Row: %d Act: %u\n",
            //                m_bank_id, row_addr, m_counters[row_addr]);
            }
        }
        }
*/
        void process_act(const Request& req) {
            auto row_addr = req.addr_vec[m_cfg.m_row_level];    
            if (m_counters.find(row_addr) == m_counters.end()) {
                m_counters[row_addr] = 0;
            }
            m_counters[row_addr]++;
            m_recently_accessed_rows.insert(row_addr); // 최근 접근한 row 추가
            if (m_debug) {
                std::printf("[ACT] [%d] Row: %d count: %u\n",
                            m_bank_id, row_addr, m_counters[row_addr]);
            }
        
            // 기존 m_critical_rows 처리 (PRAC threshold reach)
            if (m_counters[row_addr] >= m_alert_thresh) {
                std::printf("[Max] [%d] Max Row: %d Count: %u.\n", m_bank_id,row_addr,m_counters[row_addr]);
                m_critical_rows[row_addr] = m_counters[row_addr];
                m_is_abo_needed = true;
            }
            
            // 새로운 m_thres_rows 처리
            if (m_add_thres != -1 && 
                m_counters[row_addr] >= m_add_thres && 
                m_counters[row_addr] % m_add_thres == 0){
                    //m_add_thres는 service Q에 add 하기 위한 최소 조건. 그리고 배수만 반영. 16,32,48....
                
                //serviceQ  방식 추가
                if (m_serviceQ){
                if (m_thres_rows.size() >= m_max_thres_rows) {
                
                        std::printf("[Service Q full] full of service Q [%d] Max Row: %d with Count: %u.\n", m_bank_id,row_addr,m_counters[row_addr]);
                        m_critical_rows[row_addr] = m_counters[row_addr];
                        m_is_abo_needed = true;
                }
                else {
                // 새로운 row 추가
                m_thres_rows[row_addr] = m_counters[row_addr];
                if (m_debug) {
                    std::printf("[service Q add] [%d] Adding Row: %d with Count: %u\n",
                                m_bank_id, row_addr, m_counters[row_addr]);
                }
                }

                std::printf("[service Q] [%d] total :%lu \n", m_bank_id, m_thres_rows.size());
                }
            
            // adaptive_prac 방식 
            if (m_adaptive_prac){
                if (m_thres_rows.size() < m_max_thres_rows / 2 ) {
                    // m_thres_rows.size()가 m_max_thres_rows의 절반 이하 
                    m_adaptive_thres_rows = m_max_thres_rows / 2 + 1;
                    m_adapt_thres = m_add_thres;
                    if (m_debug) {
                        std::printf("[A_PRAC] [%d] [THRES] m_thres_rows is at least 50%% full (size: %lu, max: %d), adaptive_thres: %d\n",
                                    m_bank_id, m_thres_rows.size(), m_adaptive_thres_rows, m_adapt_thres);
                    }
                } else if (m_thres_rows.size() < (m_max_thres_rows * 3) / 4 ) {
                    // m_thres_rows.size()가 m_max_thres_rows의 3/4 이하
                    m_adaptive_thres_rows = (m_max_thres_rows * 3) / 4 + 1;
                    m_adapt_thres = m_add_thres * 2;
                    if (m_debug) {
                        std::printf("[A_PRAC] [%d] [THRES] m_thres_rows is at least 75%% full (size: %lu, max: %d), adaptive_thres: %d\n",
                                    m_bank_id, m_thres_rows.size(), m_adaptive_thres_rows, m_adapt_thres);
                    }
                } else if (m_thres_rows.size() < (m_max_thres_rows * 5) / 6 ) {
                    // m_thres_rows.size()가 m_max_thres_rows의 5/6 이하
                    m_adaptive_thres_rows = (m_max_thres_rows * 5) / 6 + 1;
                    m_adapt_thres = m_add_thres * 4;
                    if (m_debug) {
                        std::printf("[A_PRAC] [%d] [THRES] m_thres_rows is at least 83%% full (size: %lu, max: %d), adaptive_thres: %d\n",
                                    m_bank_id, m_thres_rows.size(), m_adaptive_thres_rows, m_adapt_thres);
                    }
                } else {
                    // m_thres_rows.size()가 m_max_thres_rows보다 작을 때
                    m_adaptive_thres_rows = m_max_thres_rows ;
                    m_adapt_thres = m_add_thres * 8;
                    if (m_debug) {
                        std::printf("[A_PRAC] [%d] [THRES] m_thres_rows is at full(size: %lu, max: %d), adaptive_thres: %d\n",
                                    m_bank_id, m_thres_rows.size(), m_adaptive_thres_rows, m_adapt_thres);
                    }
                }
                
                if (m_adapt_thres != -1 && 
                    m_counters[row_addr] >= m_adapt_thres && 
                    m_counters[row_addr] % m_adapt_thres == 0){
                    if (m_thres_rows.size() >= m_adaptive_thres_rows) {
                    std::printf("[service Q full] full of service Q [%d] Max Row: %d with Count: %u.\n", m_bank_id,row_addr,m_counters[row_addr]);
                    m_critical_rows[row_addr] = m_counters[row_addr];
                    m_is_abo_needed = true;
                    }
                    else
                    {
                    m_thres_rows[row_addr] = m_counters[row_addr];
                    std::printf("[service Q add] [%d] Row: %d with Count: %u\n",
                    m_bank_id, row_addr, m_counters[row_addr]);
                    std::printf("[service Q] [%d] total :%lu \n",m_bank_id, m_thres_rows.size());
                    }
                } 
                else
                {
                    auto min_it = std::min_element(
                        m_thres_rows.begin(), m_thres_rows.end(),
                        [](const std::pair<int, uint32_t>& a, const std::pair<int, uint32_t>& b) {
                            return a.second < b.second;
                        });
                
                    // 새로 추가할 row의 count 값이 기존의 최소 count보다 작거나 같으면 추가하지 않음
                    if (min_it != m_thres_rows.end() && m_counters[row_addr] <= min_it->second) {
                        if (m_debug) {
                            //std::printf("[service Q add] [%d] Row: %d Count: %u is not added (smaller or equal to existing minimum Count: %u)\n",
                            //            m_bank_id, row_addr, m_counters[row_addr], min_it->second);
                        }
                        return; // 추가하지 않고 종료
                    }
                
                    // 최소 count 값을 가진 항목 제거
                    if (m_debug) {
                        std::printf("[service Q remove] [%d] Removing Row with lowest Count: Row: %d, Count: %u\n",
                                    m_bank_id, min_it->first, min_it->second);
                    }
                    m_thres_rows.erase(min_it);
                  }
                }

            }
        }
    


        void process_rfm(const Request& req) {
            auto act_max = std::max_element(m_counters.begin(), m_counters.end(),
                [] (const std::pair<int, uint32_t>& p1, const std::pair<int, uint32_t>& p2) {
                    return p1.second < p2.second;
                });
            if (act_max == m_counters.end()) {
                if (m_debug) {
                    std::printf("[RFM] [%d] No critical row.\n", m_bank_id);
                }
                return;
            }
            if (m_debug) {
                std::printf("[RFM] [%d] Row: %d Act: %u\n",
                    m_bank_id, act_max->first, m_counters[act_max->first]);
            }
            m_counters[act_max->first] = 0;
            m_critical_rows.erase(act_max->first);
           
        }
    };  // class PerBankCounters

};       // clascAC

}       // namespace Ramulator
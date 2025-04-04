if (m_adapt_thres != -1 && 
    m_counters[row_addr] >= m_adapt_thres && 
    m_counters[row_addr] % m_adapt_thres == 0){
if (m_thres_rows.size() >= m_adaptive_thres_rows) {
    std::printf("[serviceQ] full of service Q [%d] Max Row: %d with Count: %u.\n", m_bank_id,row_addr,m_counters[row_addr]);
    m_critical_rows[row_addr] = m_counters[row_addr];
    m_is_abo_needed = true;
}
else
{
    m_thres_rows[row_addr] = m_counters[row_addr];
    if (m_debug) {
        std::printf("[serviceQ] [%d] Adding Row: %d with Count: %u\n",
                m_bank_id, row_addr, m_counters[row_addr]);
                std::printf("[serviceQ] [%d] total :%lu \n",m_bank_id, m_thres_rows.size());
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
            std::printf("[serviceQ] [%d] New Row: %d with Count: %u is not added (smaller or equal to existing minimum Count: %u)\n",
                        m_bank_id, row_addr, m_counters[row_addr], min_it->second);
        }
        return; // 추가하지 않고 종료
    }

    // 최소 count 값을 가진 항목 제거
    if (m_debug) {
        std::printf("[serviceQ] [%d] Removing Row with lowest Count: Row: %d, Count: %u\n",
                    m_bank_id, min_it->first, min_it->second);
    }
    m_thres_rows.erase(min_it);
  }
}
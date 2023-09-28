#ifndef COMM_PRO_H
#define COMM_PRO_H

#include <functional>
#include <vector>
#include <cstdint>
#include <cstring>

#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif

class comm_pro
{
private:
    static constexpr uint16_t max_msg_size_  = 512U;
    static uint16_t calculate_crc_(const uint8_t* data, size_t length, uint8_t type);

    enum class PackState : unsigned char
    {
        S_SYNCH_1 = 0,
        S_SYNCH_2,
        S_RX_ID,
        S_TX_ID,
        S_MSG_ID,
        S_MSG_LEN_L,
        S_MSG_LEN_H,
        S_MSG_CNT_FIRST_BYTE,
        S_MSG_CNT_SECOND_BYTE,
        S_MSG_CNT_THIRD_BYTE,
        S_MSG_CNT_FOURTH_BYTE,
        S_HEADER_CRC_L,
        S_HEADER_CRC_H,
        S_MSG_ADD,
        S_DATA_CRC_L,
        S_DATA_CRC_H,
    };

    enum class CrcType : unsigned char
    {
        HEADER ,
        DATA
    };

    uint32_t header_crc_error_{};
    uint32_t data_crc_error_{};
    uint32_t solved_pack_{};
    size_t pack_sum_{};
    uint32_t synch_error_{};
    PackState pack_state_{ PackState::S_SYNCH_1};
    uint8_t msg_size_{};
    uint8_t device_id_{};

    PACK(struct CommProPack_t
                 {
                     uint8_t synch1{};
                     uint8_t synch2{};
                     uint8_t transmitter{};
                     uint8_t receiver{};
                     uint8_t msgId{};
                     uint16_t msgLen{};
                     uint32_t msgCounter{};
                     uint8_t headerCrcL{};
                     uint8_t headerCrcH{};
                     uint8_t msg[max_msg_size_]{};
                     uint8_t dataCrcL{};
                     uint8_t dataCrcH{};
                 });

    CommProPack_t r_comm_pro_pack_;
    CommProPack_t s_comm_pro_pack_;
    std::function<void(const comm_pro*)> callback_{nullptr};

public:

    comm_pro() = default;
    explicit comm_pro(uint8_t deviceId);
    comm_pro(uint8_t deviceId_, std::function<void(const comm_pro*)> callback);
    comm_pro(const comm_pro &) = delete;
    void set_callback(std::function<void(const comm_pro *)> callback);
    const std::vector<char> get_raw_comm_pro();
    void fill_comm_pro(const uint8_t msgId, uint8_t receiver, const uint8_t msg[], uint16_t msgLen);
    void add_data(const uint8_t* data, std::size_t size);
    void solve_comm_pro(const uint8_t *data, size_t size);
    void set_device_id(uint8_t deviceId);
    uint32_t get_header_crc_error()const;
    uint32_t get_data_crc_error()const;
    uint32_t get_number_of_solved_pack()const;
    size_t get_pack_sum()const;
    uint32_t get_synch_error()const;
    uint8_t get_msg_id()const;
    const CommProPack_t &get_solved_comm_pro()const;
};

#endif // COMM_PRO_H
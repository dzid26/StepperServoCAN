#include <unity.h>

// ! copy pasted
uint8_t Msg_calc_checksum_8bit(const uint8_t *data, uint8_t len, uint16_t msg_id){
  uint16_t checksum = msg_id;
  for(uint8_t i = 0; i < len; i++){
    checksum += data[i];
  }
  checksum = (checksum & 0xFFu) + (checksum >> 8); 
  checksum &= 0xFFu;

  return (uint8_t) checksum;
}


void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}
static void test_checksum_example(void) {
    uint8_t data[8] = {0xBE, 0x12, 0x00, 0xFF, 0x4C ,0x11, 0x00, 0x1E}; // 88
    uint8_t checksum_old = data[0];
    data[0]= 0;
    uint8_t checksum = Msg_calc_checksum_8bit(data, 8, 559);
    TEST_ASSERT_EQUAL(checksum_old, checksum);
}


int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_checksum_example);
    return UNITY_END();
}

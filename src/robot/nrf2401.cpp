#include "nrf2401.h"

using namespace std;

// #include "2401.h"
uint8_t tx_frequency = 90;	//24L01频率; Freq 6: 24; Freq 8: 90
uint8_t rx_frequency = 90;	//24L01频率; Freq 6: 24; Freq 8: 90
uint8_t bandwidth = 25;  //24L01带宽

bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
static uint8_t address[2][6] = {{0x00,0x98,0x45,0x71,0x10}, {0x11,0xa9,0x56,0x82,0x21}};   // 0: Robot; 1: PC

void comm_2401::init_2401(RF24* radio) {
    // radio->powerDown();
    // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    // radio->powerUp();
    // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    // perform hardware check
    if (!radio->begin()) {
        std::cout << "radio hardware is not responding!!" << std::endl;
        return; // quit now
    }
    std::cout << "Device checked!" << std::endl;

    radio->setAutoAck(false);           // EN_AA
    radio->setChannel(tx_frequency);    // RF_CH; Freq 6: 24; Freq 8: 90
    radio->setPayloadSize(MAX_SIZE);    // RX_PW_P0
    radio->setDataRate(RF24_1MBPS);     // RF_SETUP
    radio->setPALevel(RF24_PA_HIGH);    // RF24_PA_MAX is default.
    // set the TX address of the RX node into the TX pipe
    radio->openWritingPipe(address[radioNumber]);     // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio->openReadingPipe(1, address[!radioNumber]); // using pipe 1
}

void comm_2401::config_2401(RF24* radio, uint8_t* txbuf) {
    radio->stopListening();
    int check = 0;
    for ( int i=0; i<5; i++ )
        check =  check + txbuf[i];
    check = check & 0xFF;

    if ( check == txbuf[5] ) {
        // std::cout << "2401 Config" << std::endl;
        tx_frequency = txbuf[1];
        rx_frequency = txbuf[2];
        radio->setChannel(tx_frequency);
        if ( txbuf[4] == 0x01 )
            radio->setDataRate(RF24_250KBPS);
        else if ( txbuf[4] == 0x02 )
            radio->setDataRate(RF24_1MBPS);
        else if ( txbuf[4] == 0x03 )
            radio->setDataRate(RF24_2MBPS);
    }
    else {
        std::cout << "Wrong config package" << std::endl;
    }
}
// std::string receiver_addr = "255.255.255.255";
// std::string multicast_addr = "233.233.233.233";
// asio::ip::udp::endpoint receiver_endpoint_rx;
// asio::ip::udp::endpoint multicast_ep;

// void init_udp();
// void receive();

int comm_2401::comm_2401_test() {

    int status_count = 0;
    
    while (true)
    {

        // zos::log("comm 2401 count :{}\n", status_count++);
        // TODO: restart?
        // if (status_count > 50000) {
        //     socket_rx.send_to("device on: 10.12.225.200", multicast_ep);
        //     status_count = 0;
        // }
        // status_count++;
        // uint8_t pipe;
        
        if (radio_RX.available()) {                        // is there a payload? get the pipe number that recieved it
            // uint8_t bytes = radio_RX.getPayloadSize();          // get the size of the payload
            std::scoped_lock lock(mutex_comm_2401);
            // zos::log("new pack\n");
            radio_RX.read(rxbuf, MAX_SIZE);                     // fetch payload from FIFO
            receive_flag = true;

            std::string str(rxbuf,rxbuf+MAX_SIZE);

            // socket_rx.send_to(str, receiver_endpoint_rx);


            char pAscii[MAX_SIZE];
            HexToAscii(rxbuf, pAscii, MAX_SIZE);
            std::string ascii(pAscii);
            // cout << str << endl;

            // if (radio_RX.getChannel()==24) {
            //     cout << "Freq: " << 6;
            // } else {
            //     cout << "Freq: " << 8;
            // }
            // cout << "; Receive Package: " << ascii << endl;
            // int robot_num = rxbuf[2] & 0x0f;
            // cout << "; Robot: " << robot_num << endl;
            // int Wheel[4];
            // Wheel[0] = rxbuf[6]*256 + rxbuf[7];
            // Wheel[1] = rxbuf[8]*256 + rxbuf[9];
            // Wheel[2] = rxbuf[10]*256 + rxbuf[11];
            // Wheel[3] = rxbuf[12]*256 + rxbuf[13];
            
            // cout << "Robot: " << (rxbuf[2] & 0x0f) << "    ";
            // for (int i=0; i<4; i++) {
            //     cout << "Wheel" << i <<": " << Wheel[i] << "    ";
            // }
            // cout << endl;
            
            // cout << str.length() << endl;
            // cout << "Send to: " << receiver_endpoint_rx.address().to_string() << "Receive Package: " << str << endl;
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }
    
    return 0;
}

void comm_2401::HexToAscii(unsigned char * pHex, char * pAscii, int nLen)
{
    unsigned char Nibble[2];
    for (int i = 0; i < nLen; i++)
    {
        Nibble[0] = (pHex[i] & 0xF0) >> 4;
        Nibble[1] = pHex[i] & 0x0F;
        for (int j = 0; j < 2; j++)
        {
            if (Nibble[j] < 10)
                Nibble[j] += 0x30;
            else
            {
                if (Nibble[j] < 16)
                    Nibble[j] = Nibble[j] - 10 + 'a';
            }
            *pAscii++ = Nibble[j];
        }	// for (int j = ...)
    }	// for (int i = ...)
}

bool comm_2401::get_receive_flag() {
    return receive_flag;
}

void comm_2401::set_receive_flag() {
    receive_flag = false;
}

uint8_t* comm_2401::get_rxbuf() {
    std::scoped_lock lock(mutex_comm_2401);
    return rxbuf;
}
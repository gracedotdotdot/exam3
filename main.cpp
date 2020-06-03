#include "mbed.h"

#include "mbed_rpc.h"

#include "fsl_port.h"

#include "fsl_gpio.h"

#include "MQTTNetwork.h"

#include "MQTTmbed.h"

#include "MQTTClient.h"

#define UINT14_MAX        16383

// FXOS8700CQ I2C address

#define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1) // with pins SA0=0, SA1=0

#define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1) // with pins SA0=1, SA1=0

#define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1) // with pins SA0=0, SA1=1

#define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1) // with pins SA0=1, SA1=1

// FXOS8700CQ internal register addresses

#define FXOS8700Q_STATUS 0x00

#define FXOS8700Q_OUT_X_MSB 0x01

#define FXOS8700Q_OUT_Y_MSB 0x03

#define FXOS8700Q_OUT_Z_MSB 0x05

#define FXOS8700Q_M_OUT_X_MSB 0x33

#define FXOS8700Q_M_OUT_Y_MSB 0x35

#define FXOS8700Q_M_OUT_Z_MSB 0x37

#define FXOS8700Q_WHOAMI 0x0D

#define FXOS8700Q_XYZ_DATA_CFG 0x0E

#define FXOS8700Q_CTRL_REG1 0x2A

#define FXOS8700Q_M_CTRL_REG1 0x5B

#define FXOS8700Q_M_CTRL_REG2 0x5C

#define FXOS8700Q_WHOAMI_VAL 0xC7


I2C i2c( PTD9,PTD8);

RawSerial pc(USBTX, USBRX);

RawSerial xbee(D12, D11);

int m_addr = FXOS8700CQ_SLAVE_ADDR1;


void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len);

void FXOS8700CQ_writeRegs(uint8_t * data, int len);

void getAcc();

// rpc functions & variables
void getData(Arguments *in, Reply *out);
RPCFunction rpcAcc(&getData, "getData");
char buf[256], outbuf[256];
int query_count=0;

//xbee functions & variables
void xbee_setup(void);
void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);
Thread t;
EventQueue queue;

//accel variables
uint8_t data[2] ;
int num_data=0;                           // count the number of collected accel data in each query
int tilt[100]={0};                        // store when the accelerator has been tilt>45 degree
int i=0;                                  // count the total number of collected accel data
//float X[100]={0}, Y[100]={0}, Z[100]={0}; //array for storing accel values in x,y,z dir
Thread t_velocity;
EventQueue queue_velocity;
void getVelo(void);
float X,Y,Z;
float velo=0;


int main() {

  pc.baud(115200);

  xbee_setup();

  t_velocity.start(callback(&queue_velocity, &EventQueue::dispatch_forever));
  // Get Velocity every 0.1 s
  queue_velocity.call_every(100, &getVelo);

  t.start(callback(&queue, &EventQueue::dispatch_forever));
  // Setup a serial interrupt function of receiving data from xbee
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq);


   // Enable the FXOS8700Q

  FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);

  data[1] |= 0x01;

  data[0] = FXOS8700Q_CTRL_REG1;

  FXOS8700CQ_writeRegs(data, 2);

}
void getVelo(){
  
  getAcc();
  velo = X*0.1;
  pc.printf("velocity=%f\r\n",velo);
}

void xbee_setup(){

  char xbee_reply[4];

  xbee.baud(9600);

  xbee.printf("+++");

  xbee_reply[0] = xbee.getc();

  xbee_reply[1] = xbee.getc();

  if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){

    pc.printf("enter AT mode.\r\n");

    xbee_reply[0] = '\0';

    xbee_reply[1] = '\0';

  }

  xbee.printf("ATMY 0x240\r\n");

  reply_messange(xbee_reply, "setting MY : 0x240");


  xbee.printf("ATDL 0x140\r\n");

  reply_messange(xbee_reply, "setting DL : 0x140");


  xbee.printf("ATID 0x1\r\n");

  reply_messange(xbee_reply, "setting PAN ID : 0x1");


  xbee.printf("ATWR\r\n");

  reply_messange(xbee_reply, "write config");


  xbee.printf("ATMY\r\n");

  check_addr(xbee_reply, "MY");


  xbee.printf("ATDL\r\n");

  check_addr(xbee_reply, "DL");


  xbee.printf("ATCN\r\n");

  reply_messange(xbee_reply, "exit AT mode");
}

void xbee_rx_interrupt(void)
{

  xbee.attach(NULL, Serial::RxIrq); // detach interrupt

  queue.call(&xbee_rx);

}

void xbee_rx(){

  while(xbee.readable()){

    memset(buf, 0, 256);      // clear buffer

    for(int i=0; i<255; i++) {

        char recv = xbee.getc();

        if ( recv == '\r' || recv == '\n' ) {

          xbee.printf("\r\n");

          break;

        }

        buf[i] = xbee.putc(recv);
              
    }

    RPC::call(buf, outbuf);
    
    num_data = 0;

    pc.printf("outbuf = %s\r\n", outbuf);

  }
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq);

}
void getData(Arguments *in, Reply *out){
  
  xbee.printf("%f\r\n",velo);
}

void getAcc() {

   int16_t acc16;

   uint8_t res[6];

   FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);

   acc16 = (res[0] << 6) | (res[1] >> 2);

   if (acc16 > UINT14_MAX/2)

      acc16 -= UINT14_MAX;

   X = ((float)acc16) / 4096.0f;
 
}



void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {

   char t = addr;

   i2c.write(m_addr, &t, 1, true);

   i2c.read(m_addr, (char *)data, len);

}


void FXOS8700CQ_writeRegs(uint8_t * data, int len) {

   i2c.write(m_addr, (char *)data, len);

}

void reply_messange(char *xbee_reply, char *messange){

  xbee_reply[0] = xbee.getc();

  xbee_reply[1] = xbee.getc();

  xbee_reply[2] = xbee.getc();

  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){

    pc.printf("%s\r\n", messange);

    xbee_reply[0] = '\0';

    xbee_reply[1] = '\0';

    xbee_reply[2] = '\0';

  }

}


void check_addr(char *xbee_reply, char *messenger){

  xbee_reply[0] = xbee.getc();

  xbee_reply[1] = xbee.getc();

  xbee_reply[2] = xbee.getc();

  xbee_reply[3] = xbee.getc();

  pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);

  xbee_reply[0] = '\0';

  xbee_reply[1] = '\0';

  xbee_reply[2] = '\0';

  xbee_reply[3] = '\0';

}

// int start_mqtt(){

//   wifi = WifiInterface::get_default_instance();

//       if (!wifi) {

//             printf("ERROR: No WiFiInterface found.\r\n");

//             return -1;

//       }
//       printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);

//       int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);

//       if (ret != 0) {

//             printf("\nConnection error: %d\r\n", ret);

//             return -1;

//       }

//       NetworkInterface* net = wifi;

//       MQTTNetwork mqttNetwork(net);

//       MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);


//       //TODO: revise host to your ip

//       const char* host = "192.168.0.107";

//       printf("Connecting to TCP network...\r\n");

//       int rc = mqttNetwork.connect(host, 1883);

//       if (rc != 0) {

//             printf("Connection error.");

//             return -1;

//       }

//       printf("Successfully connected!\r\n");


//       MQTTPacket_connectData data = MQTTPacket_connectData_initializer;

//       data.MQTTVersion = 3;

//       data.clientID.cstring = "Mbed";


//       if ((rc = client.connect(data)) != 0){

//             printf("Fail to connect MQTT\r\n");

//       }

//       if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){

//             printf("Fail to subscribe\r\n");

//       }

//       btn2.rise(publish_queue.event(&publish_message, &client));

//       btn3.rise(&close_mqtt);


//       int num = 0;

//       while (num != 5) {

//             client.yield(100);

//             ++num;

//       }


//       while (1) {

//             if (closed) break;

//             wait(2);

//       }


//       printf("Ready to close MQTT Network......\n");


//       if ((rc = client.unsubscribe(topic)) != 0) {

//             printf("Failed: rc from unsubscribe was %d\n", rc);

//       }

//       if ((rc = client.disconnect()) != 0) {

//       printf("Failed: rc from disconnect was %d\n", rc);

//       }


//       mqttNetwork.disconnect();

//       printf("Successfully closed!\n");


//       return 0;

// }
// void messageArrived(MQTT::MessageData& md) {

//   MQTT::Message &message = md.message;

//   char msg[300];

//   sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);

//   printf(msg);

//   wait_ms(1000);

//   char payload[300];

//   sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);

//   printf(payload);

//   ++arrivedcount;

// }


// void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {

//   message_num++;

//   MQTT::Message message;

//   char buff[100];

//   sprintf(buff, "QoS0 Hello, Python! #%d", message_num);

//   message.qos = MQTT::QOS0;

//   message.retained = false;

//   message.dup = false;

//   message.payload = (void*) buff;

//   message.payloadlen = strlen(buff) + 1;

//   int rc = client->publish(topic, message);

//   printf("rc:  %d\r\n", rc);

//   printf("Puslish message: %s\r\n", buff);

// }
// void close_mqtt() {

//   closed = true;

// }
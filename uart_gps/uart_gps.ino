//GPS is on UART3

uint8_t arr[4];

void setup() {
  // put your setup code here, to run once:
  Serial3.begin(9600); //uart3
  char Test[512]; //buffer
  char Test2[128]; //buffer
  char CLEAR[128]; //clear buffer
  char GPSTEST[11];
  char GPSTEST2[128];
  //pin24 GPS config

  //write RC1740 radio config pin high to prevent entering CONFIG mode
  //delay so user can turn on radio
  //wait one minute for gps to cold start
  sprintf(GPSTEST, "$PMTK225,0");//,0,0,1000,0,1000"); //normal operation command
  HAL_UART_Transmit(&huart4, (unsigned char*)GPSTEST, sizeof(GPSTEST), HAL_MAX_DELAY); //sends to GPS
}

void loop() {
  // put your main code here, to run repeatedly:
   __HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_ORE);
   __HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_NE);

   uret = huart4.Instance->RDR; //uart receive data register

   sprintf(Test, "%d \n\r", uret); //converts output to be displayed, WORKS AND OUTPUTS UNREADABLE DATA WHICH MEANS THAT IT IS GETTING SOMETHING
   HAL_UART_Transmit(&huart5, (unsigned char*)uret, sizeof((unsigned char*)uret), HAL_MAX_DELAY); //sends to radio

   HAL_Delay(100);
}




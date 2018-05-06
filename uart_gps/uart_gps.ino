//GPS is on UART3

uint8_t arr[4];

int main(void)
{

  char Test[512]; //buffer
  char Test2[128]; //buffer
  char CLEAR[128]; //clear buffer
  char GPSTEST[11];
  char GPSTEST2[128];
  uint16_t uret;

/* previous radio setup from stmf3 code
  //RADIO SETUP
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); //Write RC1740 Config high to prevent radio module from entering CONFIG mode.
  HAL_Delay(2500);      //user turns the radio on during this delay
  memset(CLEAR, '\0',128); //sizes buffer
  sprintf(CLEAR, "\n\r\n\r\n\r\n\r\n\r\n hey yall \r\n\r\n\r\n\r\n\r\n\r\n\r\n\r");
  HAL_UART_Transmit(&huart5, (unsigned char*)CLEAR, sizeof(CLEAR), HAL_MAX_DELAY); //send status
  HAL_Delay(1000);
*/

  /*GPS SETUP*/
  HAL_Delay(60000);
  sprintf(GPSTEST, "$PMTK225,0");//,0,0,1000,0,1000"); //normal operation command
  HAL_UART_Transmit(&huart4, (unsigned char*)GPSTEST, sizeof(GPSTEST), HAL_MAX_DELAY); //sends to GPS
  HAL_UART_Transmit(&huart5, (unsigned char*)GPSTEST, sizeof(GPSTEST), HAL_MAX_DELAY); //sends to radio


  while (1)
  {
    __HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_ORE);
    __HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_NE);

    uret = huart4.Instance->RDR; //uart receive data register

    sprintf(Test, "%d \n\r", uret); //converts output to be displayed, WORKS AND OUTPUTS UNREADABLE DATA WHICH MEANS THAT IT IS GETTING SOMETHING
    HAL_UART_Transmit(&huart5, (unsigned char*)uret, sizeof((unsigned char*)uret), HAL_MAX_DELAY); //sends to radio

    HAL_Delay(100);


    //HAL_UART_Receive(&huart4, (unsigned char*)GPSTEST2, sizeof(GPSTEST2), HAL_MAX_DELAY); //reads uart4 data

  }
}

/* previous UART inits
/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B; //this might be wrong
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 19200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
}
*/ 

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

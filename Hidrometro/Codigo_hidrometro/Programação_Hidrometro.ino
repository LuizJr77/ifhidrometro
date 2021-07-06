#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid = "redewifi";
const char* password =  "senha";
const char* mqttServer = "url_brocker";
const int mqttPort = 1883;
const char* mqttUser = "login";
const char* mqttPassword = "senha";

int pinopir = 3;  //Pino ligado ao sensor PIR
int acionamento;  //Variavel para guardar valor do sensor
int volume=1;
xTimerHandle xTimer, xTimerDHT;
QueueHandle_t xFila; /*cria objeto fila */

TaskHandle_t xTaskPrintHandle;
TaskHandle_t xTaskADCHandle;
TaskHandle_t xTaskSensorHandle;
TaskHandle_t xTaskMQTTHandle;
TaskHandle_t xTaskTecladoHandle;

void vTaskSensor(void *pvParameters ); 
void vTaskPrint(void *pvParameters);
void vTaskMQTT(void *pvParameters); 

void mqttInit();
void mqttSendJsonIO(void);
void rtosInit();
void mqttSendJsonIOviatask(void);
void setup() {
  // put your setup code here, to run once:
 rtosInit();
 pinMode (pinopir, INPUT);
}

void loop() {
  client.loop(); //atualiza conexão MQTT 
  vTaskDelay(pdMS_TO_TICKS(1000));    /* Delay de 1 segundos */

}
void rtosInit(){

    xFila = xQueueCreate(1, sizeof(int));
if (xFila == NULL)
  {
     Serial.println("Erro: nao e possivel criar a fila");
     while(1); /* Sem a fila o funcionamento esta comprometido. Nada mais deve ser feito. */
  } 
    
    xTimer = xTimerCreate("TIMER",pdMS_TO_TICKS(2000),pdTRUE, 0, callBackTimer);
    xTimerDHT = xTimerCreate("TIMER2",pdMS_TO_TICKS(10000),pdTRUE, 0, callBackTimerDHT);

    xTaskCreatePinnedToCore(
      vTaskSensor,                       /* Funcao a qual esta implementado o que a tarefa deve fazer */
       "TaskADC",                        /* Nome (para fins de debug, se necessário) */
       configMINIMAL_STACK_SIZE + 1024,  /* Tamanho da stack (em words) reservada para essa tarefa */
       NULL,                             /* Parametros passados (nesse caso, não há) */
         2,                              /* Prioridade */
           &xTaskSensorHandle,          /* Handle da tarefa, opcional (nesse caso, não há) */
           APP_CPU_NUM);                /* Core */

    xTaskCreatePinnedToCore(vTaskPrint,  "TaskPrint",  configMINIMAL_STACK_SIZE + 1024,  NULL,  1,  &xTaskPrintHandle,APP_CPU_NUM);
    xTaskCreatePinnedToCore(vTaskMQTT,  "TaskMQTT",  configMINIMAL_STACK_SIZE + 2024,  NULL,  3,  &xTaskMQTTHandle,PRO_CPU_NUM);  
    xTaskCreatePinnedToCore(vTaskTeclado,  "Taskteclado",  configMINIMAL_STACK_SIZE+1024 ,  (void *)BT1,  4,  &xTaskTecladoHandle,PRO_CPU_NUM);   

    xTimerStart(xTimer,0);
    xTimerStart(xTimerDHT,0);

    /* A partir deste momento, o scheduler de tarefas entra em ação e as tarefas executam */
}
void mqttInit(){
   
   WiFi.begin(ssid, password);

   while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Conectando ao WiFi..");
   }
  Serial.println("Conectado na rede WiFi");
}
void reconect() {
  //Enquanto estiver desconectado
  while (!client.connected()) {

   if (acionamento == HIGH){
     if (client.connect("ESP32Client", mqttUser, mqttPassword ))
    {
      Serial.println("Conectado ao broker!");
      //subscreve no tópico
      client.subscribe("hidrometro", volume); // subscreve no tópico, nivel de qualidade: QoS 1 
      
    }
    else
    {
      Serial.print("Falha na conexao ao broker - Estado: ");
      Serial.print(client.state());
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }}
}
void callback(char* topic, byte* payload, unsigned int length) {

  //armazena msg recebida em uma sring
  payload[length] = '\0';
  String strMSG = String((char*)payload);

  Serial.print("Mensagem chegou do tópico: ");
  Serial.println(topic);
  Serial.print("Mensage:");
  Serial.print(strMSG);
  Serial.println();
  Serial.println("-----------------------");

  //aciona saída conforme msg recebida
  
  mqttSendJsonIO();  // publica o status dos pinos via mqtt no broker
}
/* função que separa em json IO e envia mqtt*/
void mqttSendJsonIO(void){
    //Envia a mensagem ao broker
     /// . produzindo mensagem
    DynamicJsonDocument doc(1024);
    doc["device"] = "ESP32";
    doc["OUT1"] = digitalRead(OUTPUT_1);
    char JSONmessageBuffer[200];
    serializeJson(doc, JSONmessageBuffer);
    client.publish("esp32/out", JSONmessageBuffer);
    Serial.print("msg json out enviado: ");
    Serial.println(JSONmessageBuffer);
}
/*Implementação da Task MQTT */
void vTaskMQTT(void *pvParameters){
  (void) pvParameters;
  //char mensagem[30];
  //UBaseType_t uxHighWaterMark;   
  int valor_recebido = 0;

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  
  while(1)
  {
    
      if(xQueueReceive(xFila, &valor_recebido, portMAX_DELAY) == pdTRUE) //verifica se há valor na fila para ser lido. Espera 1 segundo
      {
        if(!client.connected()){
          reconect();
        }
        mqttSendJson(valor_recebido);   
         /* Para fins de teste de ocupação de stack, printa na serial o high water mark */
       //  uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
       //  Serial.print(pcTaskGetTaskName(NULL));
       //  Serial.print(" : ");
      //  Serial.println(uxHighWaterMark);

        vTaskDelay(5000);

      }
      else
      {
        Serial.println("TIMEOUT");
      }
 

  }

}
/* função que separa em json IO e envia mqtt por meio de task*/
void mqttSendJsonIOviatask(void){
    //Envia a mensagem ao broker
     /// . produzindo mensagem
    DynamicJsonDocument doc(1024);
    doc["device"] = "ESP32";
    doc["OUT1"] = volume;
    char JSONmessageBuffer[200];
    serializeJson(doc, JSONmessageBuffer);
    client.publish("hidrometro", JSONmessageBuffer);
    Serial.print("msg json out enviado: ");
    Serial.println(JSONmessageBuffer);
}

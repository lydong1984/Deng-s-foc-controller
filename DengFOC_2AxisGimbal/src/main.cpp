#include <Arduino.h>
#include <EEPROM.h>
#include <BMI160Gen.h>
#include <math.h>
#include "DengFOC.h"
#include "lowpass_filter.h"
#include "pid.h"

//适配v3p和v4板子   0：V3P  1：V4
#define DengFOC 0 

// 定义EEPROM模拟存储的大小（字节），ESP32建议不超过4096字节
#define EEPROM_SIZE 32
// 定义存储数据的起始地址
#define M0_Initialize_Align 0
#define M1_Initialize_Align 4
#define M0_Encoder_DIR 8
#define M1_Encoder_DIR 12

//设置报警电压
#define UNDERVOLTAGE_THRES 11.1

// 判断是否已经写入过EEPROM 1：需要写入 0：已经写入
#define Calibration_Flag 0     

LowPassFilter Y_Flt = LowPassFilter(0.1); //垂直电机滤波

PIDController Y_loop = PIDController{.P = 0.045, .I = 0.002, .D = 0.0001, .ramp = 100000, .limit =6 };//垂直电机PID

LowPassFilter Z_Flt = LowPassFilter(0.09); //水平电机滤波
LowPassFilter Z_Flt_Damping = LowPassFilter(0.1); //水平电机阻尼


PIDController Z_loop = PIDController{.P = 0.030, .I = 0.001, .D = 0.005, .ramp = 100000, .limit =6 };//水平电机PID

int ax, ay, az;       // 加速度计原始值 
int gx, gy, gz;       // 陀螺仪原始值
int bx,by,bz;         // 陀螺仪方向匹配
int cx,cy,cz;
float accX, accY, accZ;       // 换算后的加速度值（g单位）
float gyroX, gyroY,gyroZ;     // 换算后的角速度值（dps单位）
float angleAccX, angleAccY;   // 加速度计解算的角度
float interval;               // 时间间隔（秒）                                                             
unsigned long preInterval = 0; // 时间戳
float gyroXoffset = 0.0f, gyroYoffset = 0.0f,gyroZoffset = 0.0f;        // 陀螺仪偏移值（校准后赋值）
float last_angleX = 0.0f, last_angleY = 0.0f,last_angleZ = -180.0f;     // 获取的角度值


const float DEAD_ZONE = 0.05f;  // 陀螺仪死区，过滤微小噪声

float filteredGyroX = 0.0f;     // 滤波后的角速度
float filteredGyroY = 0.0f;     // 滤波后的角速度
float filteredGyroZ = 0.0f;     // 滤波后的角速度

float Torget_0 = 0;             //垂直电机目标力矩
float Torget_1 = 0;             //水平电机目标力矩
float angle1_error_0 = 0;       //垂直电机角度偏差
float angle1_error_1 = 0;       //水平电机角度偏差
float angle1_error_1_begin = 0; //判断是否静止了
int Acc = 0;                    //累加多次静止

float now = 0;                  //记录垂直电机编码器角度与陀螺仪角度偏差值

float M0_zero_electric_angle = 0;   //零电角度
float M1_zero_electric_angle = 0;

int M0_Dir = 0;               //自检极性
int M1_Dir = 0;

//获取电源电压
float GetVinVolt();

//检测电源电压
void CheckVinVolt();

//EEPROM存储FLOAT
void WriteFloatToEEPROM(int addr, float value);
void WriteIntToEEPROM(int addr, int value);

//EEPROM读取FLOAT
float ReadFloatToEEPROM(int addr);
int ReadIntToEEPROM(int addr);

//陀螺仪初始化
void Bmi160Init();

// 陀螺仪手动校准
void CalibrateGyro();

//获取陀螺仪角度
void GetAngle();

void setup() {
  // 初始化串口，用于打印调试信息
  Serial.begin(115200);
  // 等待串口初始化完成
  while (!Serial) {
    delay(10);
  }

  CheckVinVolt();   //电压检测
  DFOC_Vbus(12.6);   //设定驱动器供电电压

  Bmi160Init();
  CalibrateGyro();
    
  DFOC_enable();

  // 初始化EEPROM，设置模拟存储大小
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("EEPROM初始化失败！");
    while (1) { // 初始化失败则卡死，提示错误
      delay(1000);
    }
  }

  if(Calibration_Flag)                                      //判断是否需要记录0电角度
  {
    M0_zero_electric_angle = DFOC_M0_alignSensor(7); //M0电机，极对数和dir
    M1_zero_electric_angle = DFOC_M1_alignSensor(7); //M1电机，极对数和dir
    M0_Dir = Get_M0_Dir();
    M1_Dir = Get_M1_Dir();

    Serial.println("正在存入电角度");
    WriteFloatToEEPROM(M0_Initialize_Align, M0_zero_electric_angle);
    delay(10);
    WriteFloatToEEPROM(M1_Initialize_Align, M1_zero_electric_angle);
    delay(10);
    WriteIntToEEPROM(M0_Encoder_DIR, M0_Dir);
    delay(10);
    WriteIntToEEPROM(M1_Encoder_DIR, M1_Dir);
    
    Serial.println("存入完成");
    Serial.println("请进一步标定陀螺仪");
    while(1);
  }
  else
  {
    Serial.println("正在读取零电角度。。");
    M0_zero_electric_angle = ReadFloatToEEPROM(M0_Initialize_Align);
    delay(10);
    M1_zero_electric_angle = ReadFloatToEEPROM(M1_Initialize_Align);
    delay(10);
    M0_Dir = ReadIntToEEPROM(M0_Encoder_DIR);
    delay(10);
    M1_Dir = ReadIntToEEPROM(M1_Encoder_DIR);

    DFOC_M0_Set_alignSensor(7,M0_Dir,M0_zero_electric_angle);
    DFOC_M1_Set_alignSensor(7,M1_Dir,M1_zero_electric_angle);
    Serial.print("DIR为：");
    Serial.print(M0_Dir);
    Serial.print(",");
    Serial.println(M1_Dir);

    Serial.println("读取零电角度完成");
    Serial.print("零电角度为：");
    Serial.print(M0_zero_electric_angle);  
    Serial.print(",");
    Serial.println(M1_zero_electric_angle);
  }
  
  EEPROM.end();
  now = (DFOC_M0_Angle()*180/PI)- M0_Dir * last_angleY - M0_Dir * 90;   //记录编码器和陀螺仪的差值 -90是因为初始标定的偏差
  // now = (DFOC_M0_Angle()*180/PI)- last_angleY;   //记录编码器和陀螺仪的差值 90是因为初始标定的偏差
  GetAngle();
  Serial.println("初始化完成！");
  delay(50);
}

void loop() {
  runFOC();
  GetAngle();
  // Serial.print(last_angleX);
  // Serial.print(",");
  // Serial.print(last_angleY);
  // Serial.print(",");
  // Serial.println(last_angleZ);
  // /********************************************垂直 *********************************************/
  angle1_error_0 = M0_Dir * last_angleY - 0  + now;    //陀螺仪角度映射到目标位置

  if(fabs(angle1_error_0) <0.3)
  {
    Torget_0 = 0.0f;
  }
  else{
    Torget_0 = Y_Flt(Y_loop(angle1_error_0-(DFOC_M0_Angle()*180/PI)));
  }

    
  // Serial.print(last_angleY);
  // Serial.print(",");
  // Serial.print(now);
  // Serial.print(",");
  // Serial.print(angle1_error_0);
  // Serial.print(",");
  // Serial.print(DFOC_M0_Angle()*180/PI);
  // Serial.print(",");
  // Serial.println(angle1_error_0-(DFOC_M0_Angle()*180/PI));

  /********************************************水平 ********************************************/
  angle1_error_1 = last_angleZ - 0 ;
  if(angle1_error_1_begin ==  angle1_error_1)                       //多次静止，清空I，避免累加
  {
    Acc++;
    if(Acc > 10)
    {
      Acc = 10;
    }
  }
  else
  {
    Acc = 0;
  }

  if(Acc == 10)
  {
    Z_loop.I = 0.0;
    Torget_1 = Z_Flt(Z_loop(angle1_error_1));
    Torget_1 = 0.0f;
  }
  else{
    Z_loop.I = 0.001;
    Torget_1 = Z_Flt(Z_loop(angle1_error_1));
  }

  angle1_error_1_begin =  angle1_error_1;

  DFOC_M0_setTorque(Torget_0);
  DFOC_M1_setTorque((M1_Dir * (Torget_1 + 2.9 * sin(last_angleY * PI / 180)) + (Z_Flt_Damping(DFOC_M1_Angle()) - DFOC_M1_Angle())));                   //前馈增益+角度阻尼
  // Serial.println((M1_Dir * (Torget_1 + 2.9 * sin(last_angleY * PI / 180)) + (Z_Flt_Damping(DFOC_M1_Angle()) - DFOC_M1_Angle())));
}

/************************ 电源电压检测 ************************/
float GetVinVolt() {
  float Volts = 0;
  if(DengFOC)
  {
    Volts = analogReadMilliVolts(13) * 8.5 / 1000;
  }
  else
  {
    Volts = analogReadMilliVolts(4) * 11 / 1000;
  }

  return Volts;
}

void CheckVinVolt()
{
  float vinVolt = GetVinVolt();
  while (vinVolt <= UNDERVOLTAGE_THRES) {
    vinVolt = GetVinVolt();
    delay(100);
    Serial.printf("等待上电,当前电压%.2f\n", vinVolt);
  }
  Serial.printf("正在校准电机...当前电压%.2f\n", vinVolt);
}

/************************ 写入EEPROM ************************/
void WriteFloatToEEPROM(int addr, float value) {

  //边界检查
  if (addr < 0 || addr + 3 >= EEPROM.length()) {
    Serial.print("写入失败：地址越界！addr=");
    Serial.println(addr);
    return; // 越界则直接退出，避免错误写入
  }

  union {
    float f;
    uint8_t bytes[4];
  } floatData;
  floatData.f = value;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(addr + i, floatData.bytes[i]);
  }

  if (EEPROM.commit()) {
    Serial.println("数据写入EEPROM成功！");
  } else {
    Serial.println("数据写入EEPROM失败！");
  }

}

void WriteIntToEEPROM(int addr, int value) {

  //边界检查
  if (addr < 0 || addr + 3 >= EEPROM.length()) {
    Serial.print("写入失败：地址越界！addr=");
    Serial.println(addr);
    return; // 越界则直接退出，避免错误写入
  }

  union {
    int f;
    uint8_t bytes[4];
  } floatData;
  floatData.f = value;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(addr + i, floatData.bytes[i]);
  }

  if (EEPROM.commit()) {
    Serial.println("数据写入EEPROM成功！");
  } else {
    Serial.println("数据写入EEPROM失败！");
  }

}

/************************ 读取EEPROM ************************/
float ReadFloatToEEPROM(int addr) {

  // 边界检查：防止地址越界（假设 EEPROM 总大小为 512 字节）
  if (addr < 0 || addr + 3 >= EEPROM.length()) {
    Serial.println("EEPROM 地址越界！");
    return -1; // 返回数字，以防后续错误
  }

  union {
    float f;
    uint8_t bytes[4];
  } floatData;
  for (int i = 0; i < 4; i++) {
    floatData.bytes[i]=EEPROM.read(addr + i);
  }
  return floatData.f;
}

int ReadIntToEEPROM(int addr) {

  // 边界检查：防止地址越界（假设 EEPROM 总大小为 512 字节）
  if (addr < 0 || addr + 3 >= EEPROM.length()) {
    Serial.println("EEPROM 地址越界！");
    return -1; // 返回数字，以防后续错误
  }

  union {
    int f;
    uint8_t bytes[4];
  } floatData;
  for (int i = 0; i < 4; i++) {
    floatData.bytes[i]=EEPROM.read(addr + i);
  }
  return floatData.f;
}

/************************ 初始化陀螺仪 ************************/
void Bmi160Init()
{
  BMI160.begin(BMI160GenClass::I2C_MODE,Wire,0X69);//使能BMI，地址为0X69，默认wrie
  BMI160.autoCalibrateGyroOffset();//自动硬件校准         
  BMI160.setGyroRange(1000);  //设置量程为1000
  BMI160.setGyroRate(100);    //陀螺仪采样频率

}

/************************ 陀螺仪手动校准 ************************/
void CalibrateGyro() {
  Serial.println("开始校准陀螺仪");
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  
  // 多次采样求平均
  for (int i = 0; i < 1000; i++) {
    BMI160.readGyro(gx, gy, gz);

    //修正方向
    cx = gy;
    cy = -gx;
    cz = gz;

    // //修正方向
    // cx = gx;
    // cy = gy;
    // cz = gz;


    // 换算为dps（1000量程下，1 LSB = 1000/32768 ≈ 0.0305 dps）
    float gx_dps = (float)cx / 32.8;
    float gy_dps = (float)cy / 32.8;
    float gz_dps = (float)cz / 32.8;
    
    gx_sum += gx_dps;
    gy_sum += gy_dps;
    gz_sum += gz_dps;
    
    delay(2); // 采样间隔
  }
  
  // 计算偏移值（静止时的零漂）
  gyroXoffset = gx_sum / 1000;
  gyroYoffset = gy_sum / 1000;
  gyroZoffset = gz_sum / 1000;

  Serial.println("校准陀螺仪完成");
}

/************************ 获取角度 ************************/
void GetAngle() 
{
  BMI160.readAccelerometer(ax, ay, az);//读取加速度
  BMI160.readGyro(gx,gy,gz);        //读取角速度

  //陀螺仪方向匹配
  // bx = az;
  // by = -ax;
  // bz = -ay;
  // cx = gz;
  // cy = -gx;
  // cz = -gy;

  // bx = ax;
  // by = ay;
  // bz = az;
  // cx = gx;
  // cy = gy;
  // cz = gz;

  bx = ay;
  by = -ax;
  bz = az;
  cx = gy;
  cy = -gx;
  cz = gz;

  accX = ((float)bx) / 16384.0;//换算加速度
  accY = ((float)by) / 16384.0;
  accZ = ((float)bz) / 16384.0;

  angleAccX = atan2(accY, sqrt(accX*accX + accZ*accZ)) * 180.0 / PI;
  angleAccY = atan2(-accX, sqrt(accY*accY + accZ*accZ)) * 180.0 / PI;

  gyroX = ((float)cx) / 32.8 - gyroXoffset;//量程1000
  gyroY = ((float)cy) / 32.8 - gyroYoffset;//量程1000
  gyroZ = ((float)cz) / 32.8 - gyroZoffset;//量程1000

  // 死区处理：过滤小于0.1dps的噪声，避免积分累积
  gyroX = fabs(gyroX) < DEAD_ZONE ? 0 : gyroX;
  gyroY = fabs(gyroY) < DEAD_ZONE ? 0 : gyroY;
  // gyroZ = fabs(gyroZ) < DEAD_ZONE ? 0 : gyroZ;
   gyroZ = fabs(gyroZ) < 0.1 ? 0 : gyroZ;

  //低通滤波，角度积分
  filteredGyroX = 0.9 * filteredGyroX + (1.0f - 0.9) * gyroX;
  filteredGyroY = 0.9 * filteredGyroY + (1.0f - 0.9) * gyroY;
  filteredGyroZ = 0.98 * filteredGyroZ + (1.0f - 0.98) * gyroZ;

  // filteredGyroZ = gyroZ;

  interval = (millis() - preInterval) * 0.001;//获取系统时间

  last_angleX = (0.99f * (last_angleX + filteredGyroX * interval)) + (0.01f * angleAccX);//互补滤波
  last_angleY = (0.99f * (last_angleY + filteredGyroY * interval)) + (0.01f * angleAccY);
  if (fabs(filteredGyroZ) > 0.1f) {
  last_angleZ = last_angleZ + filteredGyroZ * interval ;
  }
  preInterval = millis();
}
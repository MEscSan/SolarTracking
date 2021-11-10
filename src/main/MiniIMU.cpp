#pragma once
#include "MiniIMU.h"

#pragma region ctor
MiniIMU::MiniIMU(OutputType outType, DeviceVersion version, AxisDefinition axis, bool correctedData) {

    _version = version;
    _axisDef = axis;
    _outType = outType;
    _correctedData = correctedData;

    // Initialize private variables
    _G_Dt = 0.02;
    _counter = 0;
    _timer24 = 0;

    if (_axisDef = AxisDefinition::Y_right_Z_Down) {
        // X axis pointing forward
        // Y axis pointing to the right
        // and Z axis pointing down.
        // Positive pitch : nose up
        // Positive roll : right wing down
        // Positive yaw : clockwise
        int sensor_Sign[9] = { 1,1,1,-1,-1,-1,1,1,1 };//Correct directions x,y,z - gyro, accelerometer, magnetometer
        
        // memmove(destination, source, size in bytes)
        memmove(_SENSOR_SIGN, sensor_Sign, 9*sizeof( int));
    }
    else {
        // X axis pointing forward
        // Y axis pointing to the left
        // and Z axis pointing up.
        // Positive pitch : nose down
        // Positive roll : right wing down
        // Positive yaw : counterclockwise
        int sensor_Sign[9] ={ 1,-1,-1,-1,1,1,1,-1,-1 };//Correct directions x,y,z - gyro, accelerometer, magnetometer
        // memmove(destination, source, size in bytes)
        memmove(_SENSOR_SIGN, sensor_Sign, 9*sizeof( int));
    }

    // Initialize MiniIMU
    I2C_Init();
    Accel_Init();
    Compass_Init();
    Gyro_Init();

    for (int i = 0; i < 32; i++)    // We take some readings...
    {
        Read_Gyro();
        Read_Accel();
        for (int y = 0; y < 6; y++)   // Cumulate values
            _AN_OFFSET[y] += _AN[y];
        delay(20);
    }

    for (int y = 0; y < 6; y++) {
        _AN_OFFSET[y] = _AN_OFFSET[y] / 32;
    }

    _AN_OFFSET[5] -= GRAVITY * _SENSOR_SIGN[5];

    Serial.println("Offset:");
    for (int y = 0; y < 6; y++) {
        Serial.println(_AN_OFFSET[y]);
    }

    _timer = millis();

}
#pragma endregion

#pragma region Compass
void  MiniIMU::Compass_Heading(){
    float MAG_X;
    float MAG_Y;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;

    cos_roll = cos(_roll);
    sin_roll = sin(_roll);
    cos_pitch = cos(_pitch);
    sin_pitch = sin(_pitch);

    // adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
    _c_magnetom_x = (float)(_magnetom_x - _SENSOR_SIGN[6] * M_X_MIN) / (M_X_MAX - M_X_MIN) - _SENSOR_SIGN[6] * 0.5;
    _c_magnetom_y = (float)(_magnetom_y - _SENSOR_SIGN[7] * M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - _SENSOR_SIGN[7] * 0.5;
    _c_magnetom_z = (float)(_magnetom_z - _SENSOR_SIGN[8] * M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - _SENSOR_SIGN[8] * 0.5;

    // Tilt compensated Magnetic filed X:
    MAG_X = _c_magnetom_x * cos_pitch + _c_magnetom_y * sin_roll * sin_pitch + _c_magnetom_z * cos_roll * sin_pitch;
    // Tilt compensated Magnetic filed Y:
    MAG_Y = _c_magnetom_y * cos_roll - _c_magnetom_z * sin_roll;
    // Magnetic Heading
    MAG_Heading = atan2(-MAG_Y, MAG_X);
}
#pragma endregion

#pragma region DCM
/**************************************************/
void  MiniIMU::Normalize(void){
    float error = 0;
    float temporary[3][3];
    float renorm = 0;

    error = -Vector_Dot_Product(&_DCM_Matrix[0][0], &_DCM_Matrix[1][0]) * .5; //eq.19

    Vector_Scale(&temporary[0][0], &_DCM_Matrix[1][0], error); //eq.19
    Vector_Scale(&temporary[1][0], &_DCM_Matrix[0][0], error); //eq.19

    Vector_Add(&temporary[0][0], &temporary[0][0], &_DCM_Matrix[0][0]);//eq.19
    Vector_Add(&temporary[1][0], &temporary[1][0], &_DCM_Matrix[1][0]);//eq.19

    Vector_Cross_Product(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20

    renorm = .5 * (3 - Vector_Dot_Product(&temporary[0][0], &temporary[0][0])); //eq.21
    Vector_Scale(&_DCM_Matrix[0][0], &temporary[0][0], renorm);

    renorm = .5 * (3 - Vector_Dot_Product(&temporary[1][0], &temporary[1][0])); //eq.21
    Vector_Scale(&_DCM_Matrix[1][0], &temporary[1][0], renorm);

    renorm = .5 * (3 - Vector_Dot_Product(&temporary[2][0], &temporary[2][0])); //eq.21
    Vector_Scale(&_DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void  MiniIMU::Drift_correction(void){
    float mag_heading_x;
    float mag_heading_y;
    float errorCourse;
    //Compensation the Roll, Pitch and Yaw drift. 
    static float Scaled_Omega_P[3];
    static float Scaled_Omega_I[3];
    float Accel_magnitude;
    float Accel_weight;

    //*****Roll and Pitch***************

    // Calculate the magnitude of the accelerometer vector
    Accel_magnitude = sqrt(_Accel_Vector[0] * _Accel_Vector[0] + _Accel_Vector[1] * _Accel_Vector[1] + _Accel_Vector[2] * _Accel_Vector[2]);
    Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
    // Dynamic weighting of accelerometer info (reliability filter)
    // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
    Accel_weight = constrain(1 - 2 * abs(1 - Accel_magnitude), 0, 1);  //  

    Vector_Cross_Product(&_errorRollPitch[0], &_Accel_Vector[0], &_DCM_Matrix[2][0]); //adjust the ground of reference
    Vector_Scale(&_Omega_P[0], &_errorRollPitch[0], Kp_ROLLPITCH * Accel_weight);

    Vector_Scale(&Scaled_Omega_I[0], &_errorRollPitch[0], Ki_ROLLPITCH * Accel_weight);
    Vector_Add(_Omega_I, _Omega_I, Scaled_Omega_I);

    //*****YAW***************
    // We make the gyro YAW drift correction based on compass magnetic heading

    mag_heading_x = cos(MAG_Heading);
    mag_heading_y = sin(MAG_Heading);
    errorCourse = (_DCM_Matrix[0][0] * mag_heading_y) - (_DCM_Matrix[1][0] * mag_heading_x);  //Calculating YAW error
    Vector_Scale(_errorYaw, &_DCM_Matrix[2][0], errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

    Vector_Scale(&Scaled_Omega_P[0], &_errorYaw[0], Kp_YAW);//.01proportional of YAW.
    Vector_Add(_Omega_P, _Omega_P, Scaled_Omega_P);//Adding  Proportional.

    Vector_Scale(&Scaled_Omega_I[0], &_errorYaw[0], Ki_YAW);//.00001Integrator
    Vector_Add(_Omega_I, _Omega_I, Scaled_Omega_I);//adding integrator to the Omega_I
}
/**************************************************/

void  MiniIMU::Matrix_update(void){
    _Gyro_Vector[0] = Gyro_Scaled_X(_gyro_x); //gyro x roll
    _Gyro_Vector[1] = Gyro_Scaled_Y(_gyro_y); //gyro y pitch
    _Gyro_Vector[2] = Gyro_Scaled_Z(_gyro_z); //gyro Z yaw

    _Accel_Vector[0] = _accel_x;
    _Accel_Vector[1] = _accel_y;
    _Accel_Vector[2] = _accel_z;

    Vector_Add(&_Omega[0], &_Gyro_Vector[0], &_Omega_I[0]);  //adding proportional term
    Vector_Add(&_Omega_Vector[0], &_Omega[0], &_Omega_P[0]); //adding Integrator term

    //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement
    if (_correctedData) {
        _Update_Matrix[0][0] = 0;
        _Update_Matrix[0][1] = -_G_Dt * _Omega_Vector[2];//-z
        _Update_Matrix[0][2] = _G_Dt * _Omega_Vector[1];//y
        _Update_Matrix[1][0] = _G_Dt * _Omega_Vector[2];//z
        _Update_Matrix[1][1] = 0;
        _Update_Matrix[1][2] = -_G_Dt * _Omega_Vector[0];//-x
        _Update_Matrix[2][0] = -_G_Dt * _Omega_Vector[1];//-y
        _Update_Matrix[2][1] = _G_Dt * _Omega_Vector[0];//x
        _Update_Matrix[2][2] = 0;
    }
    else {// Uncorrected data (no drift correction)
        _Update_Matrix[0][0] = 0;
        _Update_Matrix[0][1] = -_G_Dt * _Gyro_Vector[2];//-z
        _Update_Matrix[0][2] = _G_Dt * _Gyro_Vector[1];//y
        _Update_Matrix[1][0] = _G_Dt * _Gyro_Vector[2];//z
        _Update_Matrix[1][1] = 0;
        _Update_Matrix[1][2] = -_G_Dt * _Gyro_Vector[0];
        _Update_Matrix[2][0] = -_G_Dt * _Gyro_Vector[1];
        _Update_Matrix[2][1] = _G_Dt * _Gyro_Vector[0];
        _Update_Matrix[2][2] = 0;
    }

    Matrix_Multiply(_DCM_Matrix,_Update_Matrix, _Temporary_Matrix); //a*b=c

    for (int x = 0; x < 3; x++) //Matrix Addition (update)
    {
        for (int y = 0; y < 3; y++)
        {
            _DCM_Matrix[x][y] += _Temporary_Matrix[x][y];
        }
    }
}

void  MiniIMU::Euler_angles(void)
{
    _pitch = -asin(_DCM_Matrix[2][0]);
    _roll = atan2(_DCM_Matrix[2][1], _DCM_Matrix[2][2]);
    _yaw = atan2(_DCM_Matrix[1][0], _DCM_Matrix[0][0]);
}

#pragma endregion 

#pragma region I2C
void MiniIMU::I2C_Init(){
    Wire.begin();
}

void  MiniIMU::Gyro_Init(){
    if(_version == DeviceVersion::V5){
        // Accel_Init() should have already called gyro_acc.init() and enableDefault()
        _gyro_acc.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale
    }
    else{
        _gyro.init();
        _gyro.enableDefault();
        _gyro.writeReg(L3G::CTRL_REG4, 0x20); // 2000 dps full scale
        _gyro.writeReg(L3G::CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
    }
}

void  MiniIMU::Read_Gyro(){
    if(_version == DeviceVersion::V5){
        _gyro_acc.readGyro();
        
        _AN[0] = _gyro_acc.g.x;
        _AN[1] = _gyro_acc.g.y;
        _AN[2] = _gyro_acc.g.z;
    }
    else {
        _gyro.read();

        _AN[0] = _gyro.g.x;
        _AN[1] = _gyro.g.y;
        _AN[2] = _gyro.g.z;
    }

    _gyro_x = _SENSOR_SIGN[0] * (_AN[0] - _AN_OFFSET[0]);
    _gyro_y = _SENSOR_SIGN[1] * (_AN[1] - _AN_OFFSET[1]);
    _gyro_z = _SENSOR_SIGN[2] * (_AN[2] - _AN_OFFSET[2]);
}

void  MiniIMU::Accel_Init(){
    if(_version == DeviceVersion::V5){
        _gyro_acc.init();
        _gyro_acc.enableDefault();
        _gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
    }
    else{
        _compass.init();
        _compass.enableDefault();
        switch (_compass.getDeviceType())
        {
        case LSM303::device_D:
            _compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
            break;
        case LSM303::device_DLHC:
            _compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
            break;
        default: // DLM, DLH
            _compass.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
        }
    }
}

// Reads x,y and z accelerometer registers
void  MiniIMU::Read_Accel(){
    if(_version == DeviceVersion::V5){
        _gyro_acc.readAcc();

        _AN[3] = _gyro_acc.a.x >> 4; // shift right 4 bits to use 12-bit representation (1 g = 256)
        _AN[4] = _gyro_acc.a.y >> 4;
        _AN[5] = _gyro_acc.a.z >> 4;
    }
    else{
        _compass.readAcc();

        _AN[3] = _compass.a.x >> 4; // shift right 4 bits to use 12-bit representation (1 g = 256)
        _AN[4] = _compass.a.y >> 4;
        _AN[5] = _compass.a.z >> 4;
    }
    _accel_x = _SENSOR_SIGN[3] * (_AN[3] - _AN_OFFSET[3]);
    _accel_y = _SENSOR_SIGN[4] * (_AN[4] - _AN_OFFSET[4]);
    _accel_z = _SENSOR_SIGN[5] * (_AN[5] - _AN_OFFSET[5]);
}

void  MiniIMU::Compass_Init(){
    if (_version == DeviceVersion::V5) {
        _mag.init();
        _mag.enableDefault();
    }
    else {
        // LSM303: doesn't need to do anything because Accel_Init() should have already called compass.enableDefault()
    }
}

void  MiniIMU::Read_Compass(){
    if(_version == DeviceVersion::V5){
        _mag.read();

        _magnetom_x = _SENSOR_SIGN[6] * _mag.m.x;
        _magnetom_y = _SENSOR_SIGN[7] * _mag.m.y;
        _magnetom_z = _SENSOR_SIGN[8] * _mag.m.z;
    }
    else{
        _compass.readMag();

        _magnetom_x = _SENSOR_SIGN[6] * _compass.m.x;
        _magnetom_y = _SENSOR_SIGN[7] * _compass.m.y;
        _magnetom_z = _SENSOR_SIGN[8] * _compass.m.z;
    }
}
#pragma endregion

#pragma region Serial Output
void MiniIMU::Serial_Printdata(void){
    Serial.print("!");

    switch (_outType)
    {
        case OutputType::EULER_ANG:
            Serial.print("ANG:");
            Serial.print(ToDeg(_roll));
            Serial.print(",");
            Serial.print(ToDeg(_pitch));
            Serial.print(",");
            Serial.print(ToDeg(_yaw));
            break;
        case OutputType::DCM:
            Serial.print(",DCM:");
            Serial.print(_DCM_Matrix[0][0]);
            Serial.print(",");
            Serial.print(_DCM_Matrix[0][1]);
            Serial.print(",");
            Serial.print(_DCM_Matrix[0][2]);
            Serial.print(",");
            Serial.print(_DCM_Matrix[1][0]);
            Serial.print(",");
            Serial.print(_DCM_Matrix[1][1]);
            Serial.print(",");
            Serial.print(_DCM_Matrix[1][2]);
            Serial.print(",");
            Serial.print(_DCM_Matrix[2][0]);
            Serial.print(",");
            Serial.print(_DCM_Matrix[2][1]);
            Serial.print(",");
            Serial.print(_DCM_Matrix[2][2]);
            break;
        default:
            Serial.print(",AN:");
            Serial.print(_AN[0]);  //(int)read_adc(0)
            Serial.print(",");
            Serial.print(_AN[1]);
            Serial.print(",");
            Serial.print(_AN[2]);
            Serial.print(",");
            Serial.print(_AN[3]);
            Serial.print(",");
            Serial.print(_AN[4]);
            Serial.print(",");
            Serial.print(_AN[5]);
            Serial.print(",");
            Serial.print(_c_magnetom_x);
            Serial.print(",");
            Serial.print(_c_magnetom_y);
            Serial.print(",");
            Serial.print(_c_magnetom_z);
            break;
    }

    Serial.println();
}
#pragma endregion

#pragma region Update
void MiniIMU::Update_IMU_Values(){
    if ((millis() - _timer) >= 20)  // Main loop runs at 50Hz
    {
        _counter++;
        _timer_old = _timer;
        _timer = millis();
        if (_timer > _timer_old){
            _G_Dt = (_timer - _timer_old) / 1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
            if (_G_Dt > 0.2)
                _G_Dt = 0; // ignore integration times over 200 ms
        }
        else{
            _G_Dt = 0;
        }
        // *** DCM algorithm
        // Data adquisition
        Read_Gyro();   // This read gyro data
        Read_Accel();     // Read I2C accelerometer

        if (_counter > 5){  // Read compass data at 10Hz... (5 loop runs)
            _counter = 0;
            Read_Compass();    // Read I2C magnetometer
            Compass_Heading(); // Calculate magnetic heading
        }

        // Calculations...
        Matrix_update();
        Normalize();
        Drift_correction();
        Euler_angles();
        // ***

        Serial_Printdata();
    }

}
#pragma endregion

#pragma region Vector
//Computes the dot product of two vectors
float MiniIMU::Vector_Dot_Product(float vector1[3], float vector2[3]){
    float op = 0;

    for (int c = 0; c < 3; c++)
    {
        op += vector1[c] * vector2[c];
    }

    return op;
}

//Computes the cross product of two vectors
void MiniIMU::Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3]){
    vectorOut[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
    vectorOut[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
    vectorOut[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

//Multiply the vector by a scalar. 
void MiniIMU::Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2){
    for (int c = 0; c < 3; c++)
    {
        vectorOut[c] = vectorIn[c] * scale2;
    }
}

void MiniIMU::Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]){
    for (int c = 0; c < 3; c++)
    {
        vectorOut[c] = vectorIn1[c] + vectorIn2[c];
    }
}
#pragma endregion

#pragma region Matrix
//Multiply two 3x3 matrixs. 
void MiniIMU::Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3]){
    for (int x = 0; x < 3; x++)
    {
        for (int y = 0; y < 3; y++)
        {
            mat[x][y] = 0;

            for (int w = 0; w < 3; w++)
            {
                mat[x][y] += a[x][w] * b[w][y];
            }
        }
    }
}
#pragma endregion

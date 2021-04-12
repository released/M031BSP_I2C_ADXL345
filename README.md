# M031BSP_I2C_ADXL345
 M031BSP_I2C_ADXL345


update @ 2021/04/12

1. use I2C0 initial ADXL345 (PC1 : SCL , PC0 : SDA)

	- PIN#14 : SCL (use external pull up res.)
	
	- PIN#13 : SDA (use external pull up res.)
	
	- PIN#12 (SDO/ALT ADDRESS) : GND
	
	SDO :
	
	ALT ADDRESS pin : HIGH , 0x1D	(7BIT)	, 3A/3B (8 bit , write/read)
	
	ALT ADDRESS pin : LOW , 0x53	(7BIT)	, A6/A7 (8 bit , write/read)	
	
	- PIN#7 (CS) : HIGH
	
	- PIN#6 : VS : VDD
	
	- PIN#1 : VDDIO : VDD	
	
2. I2C_WriteData , I2C_ReadData in i2c_driver.c , is low level I2C driver interface

3. change log state per 10 sec , below is terminal output log


![image](https://github.com/released/M031BSP_I2C_ADXL345/blob/blob/master/raw_data.jpg)


![image](https://github.com/released/M031BSP_I2C_ADXL345/blob/blob/main/raw_pitch_roll_yaw.jpg)	
	
	
4. below is I2C communication with LA capture	
	
	
![image](https://github.com/released/M031BSP_I2C_ADXL345/blob/blob/master/LA_data_get_polling.jpg)


![image](https://github.com/released/M031BSP_I2C_ADXL345/blob/blob/master/LA_data_get_x.jpg)


![image](https://github.com/released/M031BSP_I2C_ADXL345/blob/blob/master/LA_data_get_y.jpg)


![image](https://github.com/released/M031BSP_I2C_ADXL345/blob/blob/master/LA_data_get_z.jpg)



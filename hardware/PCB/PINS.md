| Function                      | Connects to         | GPIO | Type                         |
|-------------------------------|---------------------|------|------------------------------|
| I2C0 SDA(Left)                      | MPU6050 + AS5600 #1 | 21   | Bus                          |
| I2C0 SCL(Left)                      | MPU6050 + AS5600 #1 | 22   | Bus                          |
| I2C1 SDA(Right)                      | AS5600 #2           | 32   | Bus                          |
| I2C1 SCL(Right)                      | AS5600 #2           | 33   | Bus                          |
| MPU6050 INT                   | MPU6050             | 34   | Input, no internal pull      |
| Driver Left EN                   | DRV8313 #1          | 4    | 3.3V                       |
| Driver Left IN1                  | DRV8313 #1          | 16   | PWM                          |
| Driver Left IN2                  | DRV8313 #1          | 17   | PWM                          |
| Driver Left IN3                  | DRV8313 #1          | 18   | PWM                          |
| Driver Left nFAULT                | DRV8313 #1          | 35   | Input, no internal pull      |
| Driver Right EN                   | DRV8313 #2          | 19   | 3.3V                       |
| Driver Right IN1                  | DRV8313 #2          | 23   | PWM                          |
| Driver Right IN2                  | DRV8313 #2          | 25   | PWM                          |
| Driver Right IN3                  | DRV8313 #2          | 26   | PWM                          |
| Driver nFAULT                | DRV8313 #1          | 39(VN)   | Input, no internal pull      |
| nSLEEP (shared, both drivers) | DRV8313 #1 + #2     | 27   | Output, kill switch           |
| Servo Left data                  | Servo #1            | 13   | PWM Data                      |
| Servo Right data                  | Servo #2            | 14   | PWM Data                      |

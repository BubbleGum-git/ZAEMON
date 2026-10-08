| Function                      | Connects to         | GPIO | Type                         |
|-------------------------------|---------------------|------|------------------------------|
| I2C0 SDA                      | MPU6050 + AS5600 #1 | 21   | Bus                          |
| I2C0 SCL                      | MPU6050 + AS5600 #1 | 22   | Bus                          |
| I2C1 SDA                      | AS5600 #2           | 32   | Bus                          |
| I2C1 SCL                      | AS5600 #2           | 33   | Bus                          |
| MPU6050 INT                   | MPU6050             | 34   | Input, no internal pull      |
| Driver A EN                   | DRV8313 #1          | 4    | 3.3V                       |
| Driver A IN1                  | DRV8313 #1          | 16   | PWM                          |
| Driver A IN2                  | DRV8313 #1          | 17   | PWM                          |
| Driver A IN3                  | DRV8313 #1          | 18   | PWM                          |
| Driver A nFAULT                | DRV8313 #1          | 35   | Input, no internal pull      |
| Driver B EN                   | DRV8313 #2          | 19   | 3.3V                       |
| Driver B IN1                  | DRV8313 #2          | 23   | PWM                          |
| Driver B IN2                  | DRV8313 #2          | 25   | PWM                          |
| Driver B IN3                  | DRV8313 #2          | 26   | PWM                          |
| Driver A nFAULT                | DRV8313 #1          | 39(VN)   | Input, no internal pull      |
| nSLEEP (shared, both drivers) | DRV8313 #1 + #2     | 27   | Output, kill switch           |
| Servo A data                  | Servo #1            | 13   | PWM Data                      |
| Servo B data                  | Servo #2            | 14   | PWM Data                      |

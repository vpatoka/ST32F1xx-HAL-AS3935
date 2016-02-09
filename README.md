# ST32F1xx-HAL-AS3935
Interconnect STM32F1xx and Lihtening Sensor AS3935 using HAL and I2C

Basically its a port from Arduiono. I was using PCB made by PlayingWithFusion.com

http://www.playingwithfusion.com/productview.php?pdid=22

Its very high quality board and works very well for me. 

Description
Breakout board for the AS3935 digital lightning sensor based on the AMS reference design. Includes specially tuned antenna, SPI or I2C interfacing, and a wide 2.4V to 5.5V standard operating range. This innovative sensor is designed to interface with most current development systems and boards, including all current Arduino modules. The breakout board features an inductor (antenna) specially designed for this application, and the board ships fully calibrated. This ensures that you don’t have to write a massive back-end to support low-level IC calibration, just focus on your final application!

PlayingWithFusion provide the source code to work with AS3935. But that was for Arduino. I tsuccessfully port it to STM32F1xx.
Also I add few other subroutines, like auto tuning to get the right capacity value for the board.

Here is how output looks like:

# Board has started with 72000000 Hz Sytem Clock
# AS3935 Lightning Sensor, SEN-39001-R01: beginning boot procedure....
# AS3935: I2C Bus is OK
# AS3935 Strike Detection has been activated on MCU
# AS3935: Powered up
# AS3935: Capacitor Tuning in progress: ==================================
# AS3935: Best Capacitor Value calculated as 40
# AS3935: Powered up
# AS3935: Set up for outdoor operation
# AS3935: Disturber detection enabled
# AS3935: Capacitance set to 40, Reading it as 8 x 7
# AS3935: Manual calibration complete
# AS3935: Reg 0x00:0x1E
#         Reg 0x01:0x22
#         Reg 0x02:0xC2
#         Reg 0x03:0x0
#         Reg 0x04:0x0
#         Reg 0x05:0x0
#         Reg 0x06:0x0
#         Reg 0x07:0x3F
#         Reg 0x08:0x7
# AS3935: Strike Energy: 0x0
# AS3935: Some source of disturbance nearby


The question remaining unsolved - for some uncertain reson the AS3935 doesn't accept the values for some capacitance. Like its calculate it as 40, but even I wrote it directly, it show it as 56 (which is best value as well).

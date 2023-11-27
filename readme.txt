Axis 1:
PWMR: PA6
PWML: PA7

Yellow: PA5
Green: PA8
Home: A15
----------------------------
Axis 2:
PWMR: PB0
PWML: PB1

Yellow: PA1
Green: PB15
Home: B3
----------------------------
Axis 3:
PWMR: PB6
PWML: PB7

Yellow: PB10
Green: PB14
Home: B4
----------------------------
Axis 4:
PWMR: PB8
PWML: PB9

Yellow: PA0
Green: PB13
Home: B5
----------------------------
UART2: 
PA2-Tx: send data to hover board
PA3-Rx: Receive CMD from PC
----------------------------
UART6: 
PA11-Tx: send data to hover board
PA12-Rx:
----------------------------
cmd test on serial:

shut down: 24010101000000000000003F000000000D0A
send y = 0.5: 24000000000000000000003F000000000D0A
send x = 0.5: 240000000000003F00000000000000000D0A
send x = 0.5, y = 0.5: 240000000000003F0000003F000000000D0A
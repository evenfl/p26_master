#!/usr/bin/env python3.7

import pigpio

pi = pigpio.pi()


pi.set_mode(22, pigpio.OUTPUT)

pi.set_PWM_frequency(22, 1000)
pi.set_PWM_dutycycle(22, 0)

pi.write(17, 0)
pi.write(27, 1)



while True:
   print(pi.get_PWM_frequency(15))
#!/usr/bin/env python3
"""
Hardware diagnostic script for Wobble Bot L298N motor driver.
Tests left and right motors independently to verify wiring, direction, and PWM.
"""

import sys
import time
import lgpio

CHIP = 4

# Left Motor (L298N Channel A)
ENA = 12   # Pin 32 (PWM)
IN1 = 17   # Pin 11 (Dir)
IN2 = 18   # Pin 12 (Dir)

# Right Motor (L298N Channel B)
ENB = 13   # Pin 33 (PWM)
IN3 = 27   # Pin 13 (Dir)
IN4 = 22   # Pin 15 (Dir)

PWM_FREQ = 100
TEST_PWM = 60  # 60% duty cycle to easily overcome static friction


def main():
    print("=" * 55)
    print("  Wobble Bot Motor Hardware Test")
    print("=" * 55)
    print(f"GPIO Chip : {CHIP}")
    print(f"Left Motor: ENA={ENA} (Pin 32), IN1={IN1} (Pin 11), IN2={IN2} (Pin 12)")
    print(f"Right Motor: ENB={ENB} (Pin 33), IN3={IN3} (Pin 13), IN4={IN4} (Pin 15)")
    print("=" * 55)

    try:
        h = lgpio.gpiochip_open(CHIP)
        print("✓ Successfully opened gpiochip 4")
    except Exception as e:
        print(f"✗ Failed to open gpiochip {CHIP}: {e}")
        sys.exit(1)

    pins = [ENA, ENB, IN1, IN2, IN3, IN4]
    for pin in pins:
        try:
            lgpio.gpio_claim_output(h, pin)
        except Exception as e:
            print(f"✗ Failed to claim pin {pin}: {e}")
            lgpio.gpiochip_close(h)
            sys.exit(1)
    print("✓ Successfully claimed all motor control pins")

    def stop():
        lgpio.gpio_write(h, IN1, 0)
        lgpio.gpio_write(h, IN2, 0)
        lgpio.gpio_write(h, IN3, 0)
        lgpio.gpio_write(h, IN4, 0)
        lgpio.tx_pwm(h, ENA, PWM_FREQ, 0)
        lgpio.tx_pwm(h, ENB, PWM_FREQ, 0)

    try:
        stop()

        # Test 1: Left Motor Forward
        print("\n[1/4] Testing LEFT Motor FORWARD (60% PWM for 1.5s)...")
        lgpio.gpio_write(h, IN1, 0)
        lgpio.gpio_write(h, IN2, 1)
        lgpio.tx_pwm(h, ENA, PWM_FREQ, TEST_PWM)
        time.sleep(1.5)
        stop()
        time.sleep(0.5)

        # Test 2: Right Motor Forward
        print("[2/4] Testing RIGHT Motor FORWARD (60% PWM for 1.5s)...")
        lgpio.gpio_write(h, IN3, 0)
        lgpio.gpio_write(h, IN4, 1)
        lgpio.tx_pwm(h, ENB, PWM_FREQ, TEST_PWM)
        time.sleep(1.5)
        stop()
        time.sleep(0.5)

        # Test 3: Both Motors Forward
        print("[3/4] Testing BOTH Motors FORWARD (60% PWM for 1.5s)...")
        lgpio.gpio_write(h, IN1, 0)
        lgpio.gpio_write(h, IN2, 1)
        lgpio.gpio_write(h, IN3, 0)
        lgpio.gpio_write(h, IN4, 1)
        lgpio.tx_pwm(h, ENA, PWM_FREQ, TEST_PWM)
        lgpio.tx_pwm(h, ENB, PWM_FREQ, TEST_PWM)
        time.sleep(1.5)
        stop()
        time.sleep(0.5)

        # Test 4: Both Motors Backward
        print("[4/4] Testing BOTH Motors BACKWARD (60% PWM for 1.5s)...")
        lgpio.gpio_write(h, IN1, 1)
        lgpio.gpio_write(h, IN2, 0)
        lgpio.gpio_write(h, IN3, 1)
        lgpio.gpio_write(h, IN4, 0)
        lgpio.tx_pwm(h, ENA, PWM_FREQ, TEST_PWM)
        lgpio.tx_pwm(h, ENB, PWM_FREQ, TEST_PWM)
        time.sleep(1.5)
        stop()

        print("\n" + "=" * 55)
        print("✓ Motor hardware test completed successfully!")
        print("  - If a motor spun backwards, invert its IN1/IN2 or IN3/IN4 wires or set left_invert/right_invert to true.")
        print("  - If neither motor spun, check battery power and L298N GND connection.")
        print("=" * 55)

    except KeyboardInterrupt:
        print("\nTest cancelled by user.")
    finally:
        stop()
        lgpio.gpiochip_close(h)


if __name__ == '__main__':
    main()


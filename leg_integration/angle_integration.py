from elkapod_driver.driver import ElkapodDriver

if __name__ == "__main__":
    driver = ElkapodDriver()

    print("The angles should be in the ranges:\nJ1: (-90, 90)\nJ2: (-90, 90)\nJ3: (0, -180)")
    while True:
        try:
            angles = input("Please enter the angles for each joint separated by one space: ").split()
            theta1, theta2, theta3 = float(angles[0]), float(angles[1]), float(angles[2])
            angles = [theta1 + 90, theta2 + 90, -theta3]
            driver.send_one_leg_frame(1, [1, 1, 1], angles)
        except KeyboardInterrupt:
            break

    driver.close_conn()




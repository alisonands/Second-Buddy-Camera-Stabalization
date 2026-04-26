# more debugging

import serial
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/cu.usbserial-0001', 115200)

roll = []
pitch = []
yaw = []
omegaX = []
omegaY = []
omegaZ = []

plt.ion()
fig = plt.figure()

try:
    while True:
        # if user closes the window → break loop
        if not plt.fignum_exists(fig.number):
            print("Plot window closed.")
            break

        line = ser.readline().decode().strip()
        values = line.split(",")

        if len(values) == 6:
            try:
                r, p, y, wx, wy, wz = map(float, values)

                roll.append(r)
                pitch.append(p)
                yaw.append(y)
                omegaX.append(wx)
                omegaY.append(wy)
                omegaZ.append(wz)

                # keep last N points
                max_len = 1000
                roll[:] = roll[-max_len:]
                pitch[:] = pitch[-max_len:]
                yaw[:] = yaw[-max_len:]
                omegaX[:] = omegaX[-max_len:]
                omegaY[:] = omegaY[-max_len:]
                omegaZ[:] = omegaZ[-max_len:]

                plt.clf()

                plt.subplot(2, 1, 1)
                plt.title("Angles")
                plt.plot(roll, label="Roll")
                plt.plot(pitch, label="Pitch")
                plt.plot(yaw, label="Yaw")
                plt.legend()

                plt.subplot(2, 1, 2)
                plt.title("Angular Velocity")                
                plt.plot(omegaX, label="Omega X")
                plt.plot(omegaY, label="Omega Y")
                plt.plot(omegaZ, label="Omega Z")
                plt.legend()

                plt.pause(0.01)

            except ValueError:
                pass

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    print("Saving figure...")

    # 🖼️ save image
    plt.figure(fig.number)
    plt.savefig("tools/plot_output.png", dpi=300)

    # 💾 optional: save raw data too
    # with open("plot_data.csv", "w") as f:
    #     for i in range(len(gyro_angle_roll)):
    #         f.write(f"{gyro_angle_roll[i]},{gyro_angle_pitch[i]},{gyro_angle_yaw[i]},{roll_angle[i]},{pitch_angle[i]}\n")

    ser.close()
# from sys import stdin
#
# # start_str = "dist/lin/ang/drive_left/drive_right/tv_l/tv_r/av_l/av_r:"
# # start_str = "dist/lin/ang/drive_left/drive_right/tv_l/tv_r/av_l/av_r/x/y/theta/t_err: "
# # start_str = "fb: "
#
#
# measurement_linear = []
# measurement_angular = []
# left_voltage = []
# right_voltage = []
#
# for line in stdin:
#     if(line[:len(start_str)] == start_str):
#         try:
#             data = line.split(" ")[1:]
#             curr_measurement_linear = data[0]
#             curr_measurement_angular = data[1]
#             curr_left_voltage = data[2]
#             curr_right_voltage = data[3][:-1]
#
#             measurement_linear.append(curr_measurement_linear)
#             measurement_angular.append(curr_measurement_angular)
#             left_voltage.append(curr_left_voltage)
#             right_voltage.append(curr_right_voltage)
#         except:
#             pass
#
# def print_latex(name, l):
#     print(name, "=", "\\left[", ",".join(l), "\\right]", sep="")
#
# print_latex("v_{measure2}",measurement_linear)
# print_latex("w_{measure2}",measurement_angular)
# print_latex("V_{left2}",left_voltage)
# print_latex("V_{right2}",right_voltage)



from sys import stdin

# start_str = "dist/lin/ang/drive_left/drive_right/tv_l/tv_r/av_l/av_r:"
start_str = "dist/lin/ang/drive_left/drive_right/tv_l/tv_r/av_l/av_r/x/y/theta/t_err: "

dist = []
lin = []
ang = []
drive_left = []
drive_right = []
target_volt_left = []
target_volt_right = []
actual_volt_left = []
actual_volt_right = []

x = []
y = []
theta = []
theta_error = []

for line in stdin:
    if(line[:len(start_str)] == start_str):
        data = line.split(" ")[1:]
        curr_dist = data[0]
        data = data[1:]
        curr_lin = data[0]
        curr_ang = data[1]
        curr_drive_left = data[2]
        curr_drive_right = data[3]
        curr_target_volt_left = data[4]
        curr_target_volt_right = data[5]
        curr_actual_volt_left = data[6]
        curr_actual_volt_right = data[7]

        curr_x = data[8]
        curr_y = data[9]
        curr_theta = data[10]
        curr_theta_error = data[11][:-1]

        dist.append(curr_dist)
        lin.append(curr_lin)
        ang.append(curr_ang)
        drive_left.append(curr_drive_left)
        drive_right.append(curr_drive_right)
        target_volt_left.append(curr_target_volt_left)
        target_volt_right.append(curr_target_volt_right)
        actual_volt_left.append(curr_actual_volt_left)
        actual_volt_right.append(curr_actual_volt_right)

        x.append(curr_x)
        y.append(curr_y)
        theta.append(curr_theta)
        theta_error.append(curr_theta_error)

def print_latex(name, l):
    print(name, "=", "\\left[", ",".join(l), "\\right]", sep="")

print_latex("d_{actual}",dist)
print_latex("v_{target}",lin)
print_latex("w_{target}",ang)
print_latex("v_{actualLeft}",drive_left)
print_latex("v_{actualRight}",drive_right)
print_latex("V_{targetLeft}",target_volt_left)
print_latex("V_{targetRight}",target_volt_right)
print_latex("V_{actualLeft}", actual_volt_left)
print_latex("V_{actualRight}",actual_volt_right)

print_latex("P_{x}",x)
print_latex("P_{y}",y)
print_latex("P_{theta}", theta)
print_latex("t_{error}",theta_error)

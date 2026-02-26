from sys import stdin

# start_str = "dist/lin/ang/drive_left/drive_right/tv_l/tv_r/av_l/av_r:"
# start_str = "dist/lin/ang/drive_left/drive_right/tv_l/tv_r/av_l/av_r/x/y/theta/t_err: "
start_str = "lin/ang/drive_left/drive_right/tv_l/tv_r/av_l/av_r/x/y/theta/tx/ty/ttheta: "

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
target_x = []
target_y = []
target_theta = []

for line in stdin:
    if(line[:len(start_str)] == start_str):
        try:
            data = line.split(" ")[1:]
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
            curr_target_x = data[11]
            curr_target_y = data[12]
            curr_target_theta = data[13][:-1]

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
            target_x.append(curr_target_x)
            target_y.append(curr_target_y)
            target_theta.append(curr_target_theta)
        except:
            pass

def print_latex(name, l):
    print(name, "=", "\\left[", ",".join(l), "\\right]", sep="")

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
print_latex("P_{theta}",theta)

print_latex("P_{targetX}",target_x)
print_latex("P_{targetY}",target_y)
print_latex("P_{targetTheta}",target_theta)



# from sys import stdin
#
# start_str = "dist/lin/ang/drive_left/drive_right/tv_l/tv_r/av_l/av_r:"
#
# dist = []
# lin = []
# ang = []
# drive_left = []
# drive_right = []
# target_volt_left = []
# target_volt_right = []
# actual_volt_left = []
# actual_volt_right = []
#
# for line in stdin:
#     if(line[:len(start_str)] == start_str):
#         data = line.split(" ")[1:]
#         curr_dist = data[0]
#         data = data[1:]
#         curr_lin = data[0]
#         curr_ang = data[1]
#         curr_drive_left = data[2]
#         curr_drive_right = data[3]
#         curr_target_volt_left = data[4]
#         curr_target_volt_right = data[5]
#         curr_actual_volt_left = data[6]
#         curr_actual_volt_right = data[7][:-1]
#
#         dist.append(curr_dist)
#         lin.append(curr_lin)
#         ang.append(curr_ang)
#         drive_left.append(curr_drive_left)
#         drive_right.append(curr_drive_right)
#         target_volt_left.append(curr_target_volt_left)
#         target_volt_right.append(curr_target_volt_right)
#         actual_volt_left.append(curr_actual_volt_left)
#         actual_volt_right.append(curr_actual_volt_right)
#
# def print_latex(name, l):
#     print(name, "=", "\\left[", ",".join(l), "\\right]", sep="")
#
# print_latex("d_{actual}",dist)
# print_latex("v_{target}",lin)
# print_latex("w_{target}",ang)
# print_latex("v_{actualLeft}",drive_left)
# print_latex("v_{actualRight}",drive_right)
# print_latex("V_{targetLeft}",target_volt_left)
# print_latex("V_{targetRight}",target_volt_right)
# print_latex("V_{actualLeft}", actual_volt_left)
# print_latex("V_{actualRight}",actual_volt_right)

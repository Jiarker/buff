in_length = int(input())
in_data = []
for i in range(0, in_length):
    temp = [int(x) for x in input().split()]
    in_data.append(temp)

if in_length < 3:
    print(-1, -1)
    exit(0)
temp_useinfo = []
for i in range(1, in_length - 1):
    for j in range(1, in_length - 1):
        if in_data[i][j] > in_data[i - 1][j] and in_data[i][j] > in_data[i][j - 1] \
                and in_data[i][j] > in_data[i + 1][j] and in_data[i][j] > in_data[i][j + 1]:
            temp_useinfo.append([in_data[i][j], i, j])
max_temp = temp_useinfo[0][0]
max_out = []
for each_info in temp_useinfo:
    if each_info[0] >= max_temp:
        max_out = [each_info[1], each_info[2]]

if max_out:
    print(max_out[0] + 1, max_out[1] + 1)
else:
    print(-1, -1)

in_arr = input().split()
in_data = [int(x) for x in input().split()]

store_info = [0, 0]
for i in range(0, int(in_arr[0])):
    total_temp = 0
    for temp in in_data[i:i + int(in_arr[1]) + 1]:
        total_temp += temp
    if total_temp > store_info[0]:
        store_info[0] = total_temp
        store_info[1] = i + 1

print(store_info[0], store_info[1])

# temp = input()
# N = int(temp.split(' ')[0])
# M = int(temp.split(' ')[1])

in_text = input()
out_answer = []
try:
    in_data = in_text.split(" ")
    if len(in_data) >= 2:
        for in_loop in range(int(in_data[0]) + 1, int(in_data[1])):
            if in_loop % 5 == 0:
                out_answer.append(in_loop)
            if len(out_answer) == 3:
                break
        while len(out_answer) < 3:
            out_answer.append(-1)
except IOError:
    print(-1, -1, -1)
finally:
    print(out_answer[0], out_answer[1], out_answer[2])

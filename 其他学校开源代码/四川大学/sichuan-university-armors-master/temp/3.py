import re

mystr = input()
out = []
test = re.finditer('_1234\.exe', mystr)
for i in test:
    out.append(i.span()[0])
out = [len(out)] + [x for x in out]
for i in out:
    print(i)

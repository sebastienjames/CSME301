print()

for step_limit in range(16, 19, 1):
    for stride in range(2, 30-step_limit, 4):
        for stride_count in range(3, 10, 2):
            print(step_limit, stride, stride_count)
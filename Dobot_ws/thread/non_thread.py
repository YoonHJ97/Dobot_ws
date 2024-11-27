def task1():
    for i in range(10):
        print("Thread 1 : ", i)
def task2():
    for i in range(10):
        print("Thread 2 : ", i)


task1()
task2()

print("모든 작업 완료")


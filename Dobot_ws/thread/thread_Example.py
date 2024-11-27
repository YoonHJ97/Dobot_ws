import threading

def task1():
    for i in range(10):
        print("Thread 1 : ", i)
def task2():
    for i in range(10):
        print("Thread 2 : ", i)

# Thread 생성
thread1 = threading.Thread(target=task1)
thread2 = threading.Thread(target=task2)

# Thread 실행
thread1.start()
thread2.start()

# Thread 완료 대기
thread1.join()
thread2.join()

print("모든 작업 완료")


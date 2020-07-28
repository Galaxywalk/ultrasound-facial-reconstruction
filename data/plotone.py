import numpy as np
import matplotlib.pyplot as plt
# name = "./PGA460-EDD 7-26-2020 18_54_59_277.txt"    # test
# name = "./PGA460-EDD 7-26-2020 21_26_0_600.txt"     # normal
# name = "./PGA460-EDD 7-26-2020 21_26_19_124.txt"    # open mouth


# name = "./PGA460-EDD 7-26-2020 21_48_56_723.txt"    # normal
# name = "./PGA460-EDD 7-26-2020 21_49_2_674.txt"     #|O:|
# name = "./PGA460-EDD 7-26-2020 21_49_9_779.txt"     #Puff cheeks
# name = "./PGA460-EDD 7-26-2020 21_49_15_970.txt"    #Grin
name = "./PGA460-EDD 7-26-2020 21_49_25_730.txt"    #Stare

def initial():
    f = open(name)
    line = f.readline()

    total = [[n for n in range(0,128)],[0 for n in range(0,128)]]
    print(total)
    for i in range(0,10):
        M = []
        line = f.readline()
        while line:
            line = line.strip('\n')
            line = line.split(',')
            line = line[:-1] #get rid of ' '
            line = list(map(float,line))
            M.append(line)
            # print(line)
            if line[0] == 127+i*128:
                # print("-----------------------")
                break
            line = f.readline()
        # M = M[:-2]  # get rid of space line
        M = np.array(M)
        M = M.T
        plt.figure(1)
        plt.xlim((0,128))
        x = M[0]

        # print(M[2])
        for single in range(0,128):
            x[single] = x[single]-128*i
        # print(x)
        for n in range(0,128):
            total[1][n] = total[1][n]*0.1+M[2][n]
        plt.plot(x,M[2],alpha=0.4,color = 'blue')
        # plt.savefig(".\\"+str(i)+".png")
        # plt.show()
        # print(M)
    plt.figure(1)
    # plt.savefig(".\\grin.png")
    f = open(".\\all.txt", "a")
    for number in range(0,128):
        f.writelines(str(total[1][number])+'\n')
    plt.plot(total[0],total[1],alpha=0.4,color = 'red')
    plt.show()
    return M


def plot_it():
    M = initial()
    plt.figure(1)
    plt.xlim((0,128))
    plt.plot(M[0],M[2],alpha=0.4,color = 'red')
    plt.show()

# plot_it()
initial()


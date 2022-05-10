from .white_noize_gen import gen_colour_noize
from matplotlib import pyplot as plt
from numpy import linspace, mean, rad2deg

def data_gen(rnd:bool,
    acc_offset, gyr_drift,
    a, w,
    sample_time, data_frequency,
    Ta=0, Tg=0,
    sigma_a=0, sigma_g=0,
    plot = False,
    ):
    if rnd:
        A_X = gen_colour_noize(sigma_a, Ta, sample_time, data_frequency)
        A_Y = gen_colour_noize(sigma_a, Ta, sample_time, data_frequency)
        A_Z = gen_colour_noize(sigma_a, Ta, sample_time, data_frequency)

        G_X = gen_colour_noize(sigma_g, Tg, sample_time, data_frequency)
        G_Y = gen_colour_noize(sigma_g, Tg, sample_time, data_frequency)
        G_Z = gen_colour_noize(sigma_g, Tg, sample_time, data_frequency)


        # добавляем к случайному составляющей измеряемое значение и дрейф
        A_X = [acc + acc_offset[0]+a[0] for acc in A_X]
        A_Y = [acc + acc_offset[1]+a[1] for acc in A_Y]
        A_Z = [acc+a[2] for acc in A_Z]

        G_X = [gyr+gyr_drift[0]+w[0] for gyr in G_X]
        G_Y = [gyr+gyr_drift[1]+w[1] for gyr in G_Y]
        G_Z = [gyr+w[2] for gyr in G_Z]


        if plot:
            size = (210/25.4, 297/25.4)
            fig1,axs1 = plt.subplots(6,2,constrained_layout=True, sharex='col')
            fig1.set_size_inches(size)

            # строим автокорреляцию
            lags=4
            axs1[0,1].set_title("Автокорреляция")
            axs1[0,1].acorr(A_X,
                usevlines=False, maxlags=lags, linestyle="solid", marker="");
            axs1[1,1].acorr(A_Y,
                usevlines=False, maxlags=lags, linestyle="solid", marker="");
            axs1[2,1].acorr(A_Z,
            usevlines=False, maxlags=lags, linestyle="solid", marker="");
            axs1[3,1].acorr(G_X,
            usevlines=False, maxlags=lags, linestyle="solid", marker="");
            axs1[4,1].acorr(G_Y,
            usevlines=False, maxlags=lags, linestyle="solid", marker="");
            axs1[5,1].acorr(G_Z,
            usevlines=False, maxlags=lags, linestyle="solid", marker="");

            #''' графики случайного сигнала
            axs1[0,0].set_title("Сигнал акселерометров")
            axs1[3,0].set_title("Сигнал гироскопов")

            x_axis = linspace(0, sample_time, len(A_X))

            axs1[0, 0].plot(x_axis, A_X)
            axs1[0,0].set_ylabel("x, м/c/c")
            axs1[1, 0].plot(x_axis, A_Y)
            axs1[1,0].set_ylabel("y, м/c/c")
            axs1[2, 0].plot(x_axis, A_Z)
            axs1[2,0].set_ylabel("z, м/c/c")
            axs1[3, 0].plot(x_axis, rad2deg(G_X))
            axs1[3,0].set_ylabel("x, град/c")
            axs1[4, 0].plot(x_axis, rad2deg(G_Y))
            axs1[4,0].set_ylabel("y, град/c")
            axs1[5, 0].plot(x_axis, rad2deg(G_Z))
            axs1[5,0].set_ylabel("z, град/c")
            axs1[5,0].set_xlabel("время, c");
            fig1.savefig("./images/"+"Сигналы датчиков"+".jpg", bbox_inches='tight')

            print("X: ", mean(A_X), "\n", "Y:", mean(A_Y), "\n", "Z:", mean(A_Z), "\n",
                "X:", mean(rad2deg(G_X)), "\n", "Y:", mean(rad2deg(G_Y)), "\n", "Z:", mean(rad2deg(G_Z)), "\n")
    else:
        #%% Сигнал датчиков без учета случайной составляющей
        G_X = w[0]+gyr_drift[0];
        G_Y = w[1]+gyr_drift[1];
        G_Z = w[2]
        A_X = a[0] + acc_offset[0]
        A_Y = a[1] + acc_offset[1]
        A_Z = a[2]

    return A_X, A_Y, A_Z, G_X, G_Y, G_Z
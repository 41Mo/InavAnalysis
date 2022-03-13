def aperiodic_filter(input, output_array, k, T, tau):
    if len(output_array) == 0:
        output_array.append(k*tau/(T+tau)*input +
                                T/(T+tau)*input)
    else:
        output_array.append(k*tau/(T+tau)*input + 
                                T/(T+tau)*output_array[len(output_array)-1])

def filter(input, time):
    k = 1
    T = 60
    tau = 0.01

    result = []
    for value in input:
        aperiodic_filter(value, result, k, T, tau)
    return result

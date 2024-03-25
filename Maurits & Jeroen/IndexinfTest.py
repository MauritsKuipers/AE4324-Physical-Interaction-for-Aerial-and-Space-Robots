claw_pos = {
    'time': [0, 2, 4],
    'claw': [0, 1, 0]
}

for t in range(6):
    ind_list = [claw_pos['time'].index(i) for i in claw_pos['time'] if i <= t]
    if len(ind_list) > 0:
        claw_index = ind_list[-1]
    else:
        claw_index = 0

    print(claw_pos['claw'][claw_index])
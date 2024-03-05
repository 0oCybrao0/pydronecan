import random
import time

a = [i for i in range(10000)]
for i in range(4222 - 475):
    a[i] = 1.0
for i in range(4222 - 475, 4222):
    a[i] = 1.5
for i in range(4222, 10000):
    a[i] = 0.0
# Blackjack winrate is 42.22%, getting 21 is 4.75% with 1.5x payout

PROFIT = 5
bucket = 1
bet = PROFIT

random.seed(time.time())
random.shuffle(a)
longest_loss = 0
longest_win = 0
loss, win = 0, 0
for i in range(1000):
    if a[i] > 0:
        if loss > longest_loss:
            longest_loss = loss
        loss = 0
        win += 1
    else:
        if win > longest_win:
            longest_win = win
        win = 0
        loss += 1
print("Longest loss:", longest_loss)
print("Longest win:", longest_win)

for i in range(20):
    bucket = bet * (2 ** i)
    print("Round", i + 1, "bucket:", bucket)
    for j in range(10000):
        if a[j] == 1.0:
            bucket += bet
            bet = PROFIT
        elif a[j] == 1.5:
            bucket += bet * 1.5
            bet = PROFIT
        else:
            bucket -= bet
            bet *= 2
            # print("Lost", bet, "at", j + 1, "th game")
        if bucket <= 0:
            print("Bankrupt in round", i + 1, "at", j + 1, "th game")
            bet = PROFIT
            bucket = 1
            break
    if bucket > 0:
        print("Profit:", bucket - PROFIT * (2 ** i))
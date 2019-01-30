def double_number(number):
    while True:
        yield 1
        yield 2
        yield 3

c = double_number(2)
next(c)
c.send(0)
c.send(None)
c.send(10)

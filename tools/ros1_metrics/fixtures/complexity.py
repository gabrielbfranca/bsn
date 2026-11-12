class Branch:
    def linear(self, x):
        return x
    def branch(self, x):
        if x > 0:
            return x
        return -x

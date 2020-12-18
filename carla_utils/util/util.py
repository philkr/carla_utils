class Printable:
    def __str__(self):
        args = ' '.join(['{}={!s}'.format(n, getattr(self, n)) for n in vars(self)])
        return '<{!s} {}>'.format(self.__class__.__name__, args)

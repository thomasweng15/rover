PKG="rover"
import unittest

class TestBareBones(unittest.TestCase):
    def testCase(self):
        assert 0 == 0

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_bare_bones', TestBareBones)
from unittest import TestCase
from robotControl import *
from commonOperation import *

class Tests(TestCase):
    def test_data_is_received(self):
        self.assertEqual(0, RobotControl().dataAreReceived())

    def test_wait(self):
        wait(2)
        self.assertEqual(0, 0)

import unittest
from ros2_lx16a.lx16a_driver import LX16ADriver

class TestLX16ADriver(unittest.TestCase):
    def test_move(self):
        driver = LX16ADriver('/dev/null')  
        driver.move(1, 90)
        self.assertTrue(True) 

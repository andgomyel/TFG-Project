import sys
import os
import threading
import smach

from robot_controller import robot_controller

#### Maquina de estados ####

class get_key_input(threading.Thread):

    done = False
    key = ""

    def __init__(self):
        super(get_key_input, self).__init__()
        self.daemon = True

        self.start() # Iniciamos la ejecucion de la funcion run() en un hilo a parte.

    def run(self):
        while not self.done:
            self.key = input().upper()

class default_machine(smach.State):

    transitions = {}
    
    run_event = threading.Event()


    n_robots = 3
    robots = []

    for i in range(n_robots):
        robots.append(robot_controller(i, run_event))

    user_input = get_key_input()

    def threads_management(self, threads):
        self.run_event.set()
        for t in threads:
            t.start()

        while not False in [t.is_alive() for t in threads]:
            if self.user_input.key.upper() in self.transitions.keys():
                self.key = self.user_input.key.upper()
                self.run_event.clear()
                break

        self.run_event.clear()

        for t in threads:
            t.join()

    def check_valid_input(self):
        key = self.user_input.key

        if key in self.transitions.keys():
            self.key = key
            self.user_input.key = ""
            return True


        return False

    def execute(self, userdata):
        try:
            while not self.check_valid_input():
                if not self.main(): break
        except KeyboardInterrupt:
            self.key = "Q"

        return self.key

    def main(self): None


class Wait(default_machine):

    transitions = { 'Q': 'exit',
                    'I': 'INITIALIZE', 'H': 'GO HOME'}

    def __init__(self):
        smach.State.__init__(self, outcomes=self.transitions.keys())

    def main(self):

        threads = []

        for i in range(len(self.robots)):
            thread = threading.Thread(target=self.robots[i].WAIT, args=())
            threads.append(thread)

        self.threads_management(threads)

        for robot in self.robots:
            robot.new_init_pose = True
            robot.init_poses = []
            
        return True

class Initialize(default_machine):

    transitions = { 'Q': 'exit', 
                    'W': 'WAIT', 'E': 'EXPLORE'}

    def __init__(self):
        smach.State.__init__(self, outcomes=self.transitions.keys())

    def main(self):
        
        self.key = "E" # default outcome.
        
        threads = []

        for i in range(len(self.robots)):
            thread = threading.Thread(target=self.robots[i].INITIALIZE, args=())
            threads.append(thread)

        self.threads_management(threads)
        
        init_pose_reached = []
        for robot in self.robots:
            init_pose_reached.append(robot.ready)
        
        if all(init_pose_reached):
            for robot in self.robots:
                robot.new_init_pose = True
                robot.init_poses = []

        return not all(init_pose_reached) 

class Explore(default_machine):

    transitions = { 'Q': 'exit', 
                    'W': 'WAIT', 'H': 'GO HOME'}

    def __init__(self):
        smach.State.__init__(self, outcomes=self.transitions.keys())

    def main(self):
        self.key = "W"  # Default outcome.

        threads = []

        for i in range(len(self.robots)):
            thread = threading.Thread(target=self.robots[i].EXPLORE, args=())
            threads.append(thread)

        self.threads_management(threads)
    
        return False

class GoHome(default_machine):
    enter_homing = True

    transitions = { 'Q': 'exit', 
                    'W': 'WAIT'}

    def __init__(self):
        smach.State.__init__(self, outcomes=self.transitions.keys())

    def main(self):
        self.key = "W"  # Default outcome.
        threads = []

        if self.enter_homing:
            self.enter_homing = False
            print("[#] Homing robots...")

        for i in range(len(self.robots)):
            thread = threading.Thread(target=self.robots[i].HOMING, args=())
            threads.append(thread)

        self.threads_management(threads)
    
        home_reached = []
        for robot in self.robots:
            home_reached.append(robot.goal_achieved)

        return not all(home_reached)    # Devuelve falso cuando todos los robots han llegado, por lo que saldría del estado


class State_Machine():
    
    def __init__(self):
        super().__init__()
        sys.stdout = open(os.devnull, 'w')          # Desactivar la monitorización por el terminal para que no aparezcan los comandos de la inicialización de la maquina.
        self.state_machine = smach.StateMachine(outcomes=['exit'])  # Definimos el objeto de la maquina de estados.
        
    def run(self):
        with self.state_machine:
            smach.StateMachine.add('WAIT',  Wait(),     transitions=Wait.transitions)
            smach.StateMachine.add('INITIALIZE', Initialize(),    transitions=Initialize.transitions)
            smach.StateMachine.add('EXPLORE', Explore(),    transitions=Explore.transitions)
            smach.StateMachine.add('GO HOME', GoHome(), transitions=GoHome.transitions)

            sys.stdout = sys.__stdout__     # Reactivamos los prints en el terminal.
            self.state_machine.execute()         # Iniciamos maquina de estados.
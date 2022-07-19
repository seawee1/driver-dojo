import gym
import sys
import numpy as np
import queue as Queue
from tkinter import Button, Frame, Tk
import threading
import driver_dojo

eventQueue = Queue.Queue()


def left_key(event):
    eventQueue.put("left")


def right_key(event):
    eventQueue.put("right")


def up_key(event):
    eventQueue.put("up")


def down_key(event):
    eventQueue.put("down")


def space(event):
    eventQueue.put("space")


def w(event):
    eventQueue.put("w")


def a(event):
    eventQueue.put("a")


def s(event):
    eventQueue.put("s")


def d(event):
    eventQueue.put("d")


def q(event):
    eventQueue.put("q")


def e(event):
    eventQueue.put("e")


def f(event):
    eventQueue.put("f")


def g(event):
    eventQueue.put("g")


def h(event):
    eventQueue.put("h")


def r(event):
    eventQueue.put("r")


class Client:
    def __init__(self, master):
        self.master = master
        self.running = True
        self.thread = threading.Thread(target=self.workerThread)
        self.thread.start()
        self.periodicCall()

    def periodicCall(self):
        if not self.running:
            sys.exit(1)
        self.master.after(1, self.periodicCall)

    def workerThread(self):
        env = gym.make(env_name, config=config)
        obs = env.reset()
        global eventQueue
        try:
            while True:
                action = len(key_action_map.keys())
                if eventQueue.qsize() == 0:
                    pass
                if eventQueue.qsize():
                    try:
                        key = eventQueue.get(0)
                        if key == "r":
                            env.reset()
                            continue
                        elif key in key_action_map.keys():
                            action = key_action_map[key]
                        eventQueue = Queue.Queue()
                    except Queue.Empty:
                        pass
                obs, reward, done, info = env.step(np.array(action))

                if done:
                    obs = env.reset()

        finally:
            env.close()
            self.running = False


if __name__ == "__main__":
    from omegaconf import OmegaConf

    conf = OmegaConf.from_cli()
    if "env" in conf:
        env_name = conf.env
        del conf.env
    else:
        # Override this to change the environment
        env_name = "DriverDojo/Sem-TPS-Intersection-v0"

    # Default configuration for manual control
    config = dict(
        simulation=dict(
            dt=0.03,
            max_time_steps=100000,
            stand_still_timeout_after=100000,
            init_time=0.0,
            render=True,
            render_navigation=True,
            render_track_ego=True,
            info=True,
        ),
    )

    if "Sem" in env_name:
        config["actions"] = dict(
            space="Semantic",
            extended_road_options=True,
            hie_accel=True,
            disc_hie_cross_prod=False,
        )
        print("Click on the button and use the following controls:")
        print(
            "A: Switch-left | D: Switch-right | W: Accelerate | S: Break | Q: Follow-left at junction | R: Follow-right at junction"
        )
        # 0: decel, 1: accel, 2: switchleft, 3: switchright, 4:follow, 5: left, 6:right, 7: noop
        key_action_map = dict(s=0, w=1, a=2, d=3, f=4, q=5, e=6)
    elif "Disc" in env_name:
        config["vehicle"] = dict(caster_effect=True,)
        config["actions"] = dict(
            space="Discretized", disc_dimensions=[2, 2], disc_hie_cross_prod=False,
        )
        print("Click on the button and use the following controls:")
        print("A: Left-steer | D: Right-steer | W: Accelerate | S: Break")
        # 0: full-right steering, 1: full-left steering, 2: full-decel, 4: full-accel, 5: noop
        key_action_map = dict(d=0, a=1, s=2, w=3)
    else:
        raise ValueError(
            "Only 'Sem' and 'Disc' environments are supported for manual controlling!"
        )

    # Overwrite based config with CLI arguments
    config = OmegaConf.merge(config, conf)

    root = Tk()
    root.geometry("250x100+0+0")
    frame = Frame(root)
    Button(frame, text="Click here before using the keyboard...").grid(row=0)
    root.bind("<w>", w)
    root.bind("<s>", s)
    root.bind("<r>", r)
    root.bind("<a>", a)
    root.bind("<d>", d)
    root.bind("<f>", f)
    root.bind("<g>", g)
    root.bind("<h>", h)
    root.bind("<q>", q)
    root.bind("<e>", e)

    root.winfo_screenwidth()
    root.winfo_screenheight()

    frame.pack()

    Client(root)
    root.mainloop()

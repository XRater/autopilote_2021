from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)
        self.yellow_lower = np.array([210, 200, 0], dtype="uint8")
        self.yellow_upper = np.array([255, 255, 50], dtype="uint8")

    def solve(self):
        env = self.generated_task['env']
        img, _, _, _ = env.step([0, 0])

        self.move_forward(env, img, self.create_duck_threshold_checker(high=0.015))
        self.move_left(env)
        self.move_forward(env, img, self.create_duck_threshold_checker(low=0.001))
        self.move_right(env)
        for _ in range(10):
            img, reward, done, info = env.step([1, 0])
            env.render()

        env.step([0, 0])

    def move_forward(self, env, img, condition=None):
        while condition(img):
            img, reward, done, info = env.step([1, 0])
            env.render()

    def move_left(self, env):
        for _ in range(18):
            img, reward, done, info = env.step([0.4, 1])
            env.render()
        for _ in range(18):
            img, reward, done, info = env.step([0.4, -1])
            env.render()

    def move_right(self, env):
        for _ in range(18):
            img, reward, done, info = env.step([0.4, -1])
            env.render()
        for _ in range(18):
            img, reward, done, info = env.step([0.4, 1])
            env.render()

    def duck_threshold(self, img, low=0, high=1):
        w, h, c = img.shape
        mask = cv2.inRange(img, self.yellow_lower, self.yellow_upper)
        percent = np.ones_like(mask)[mask == 255].sum() / (w * h)
        return low <= percent and percent <= high

    def create_duck_threshold_checker(self, low=0, high=1):
        return lambda img: self.duck_threshold(img, low, high)
from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)
        self.yellow_lower = np.array([210, 200, 0], dtype="uint8")
        self.yellow_upper = np.array([255, 255, 50], dtype="uint8")
        self.max_yellow_percent = 0.03

    def solve(self):
        env = self.generated_task['env']
        img, _, _, _ = env.step([0, 0])

        condition = True
        while condition:
            img, reward, done, info = env.step([1, 0])
            is_duck_close = self.check_duck(img)
            condition = not(is_duck_close)
            env.render()

        env.step([0, 0])

    def check_duck(self, img):
        w, h, c = img.shape
        mask = cv2.inRange(img, self.yellow_lower, self.yellow_upper)
        percent = np.ones_like(mask)[mask == 255].sum() / (w * h)
        return percent > self.max_yellow_percent

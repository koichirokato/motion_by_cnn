import cv2
import random
import numpy as np

from tensorflow.keras.preprocessing.image import img_to_array


def predict(img_np, model, labels) -> str:
    img = cv2.resize(img_np, (64, 64))
    img = img_to_array(img)
    X = []
    X.append(img)
    X = np.asarray(X)
    X = X / 255.0
    preds = model.predict(X)
    pred_label = ""

    label_num = 0
    tmp_max_pred = 0
    for i in preds[0]:
        if i > tmp_max_pred:
            pred_label = labels[label_num]
            tmp_max_pred = i
        label_num += 1

    return pred_label


def make_cmdvel(label) -> (float, float, float, str):
    next_motion = ""
    x, y, heading = 0.0, 0.0, 0.0
    if label == "go":
        next_motion = "go"
        x = 0.1
    else:
        random_data = random.randint(0, 2)
        if random_data == 0:
            next_motion = "turn right"
            heading = -1.0
        elif random_data == 1:
            next_motion = "turn left"
            heading = 1.0
        else:
            next_motion = "back"
            x = -0.1

    return x, y, heading, next_motion

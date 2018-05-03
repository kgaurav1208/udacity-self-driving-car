import numpy as np
import keras
import matplotlib.pyplot as plt
import csv
import cv2
import sklearn
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from keras.models import *
from keras.layers import *
from keras.optimizers import Adam



def load_data(file_path):
    
    samples = []

    # read data from csv file
    with open(file_path,'rt') as f:
        reader = csv.reader(f)
        for line in reader:
            samples.append(line)
    labels = samples.pop(0)
    return samples, labels
    


def augment_data(samples, correction = 0.2):

    augmented_samples = []

    for i in range(len(samples)):
        
        # image flipping will be done while reading image data in generator
        # here, only the angle is flipped, and a tag is added
        
        for flipped in [-1,1]:
            
            # center image
            augmented_samples.append([samples[i][0], flipped*float(samples[i][3]), flipped])
            
            # left image
            augmented_samples.append([samples[i][1], flipped*(float(samples[i][3])+correction), flipped])
            
            # right image
            augmented_samples.append([samples[i][2], flipped*(float(samples[i][3])- correction), flipped])
            
    return augmented_samples



def image_preprocessing(img):
    
    # downscale the image
    image = cv2.resize(img, (64, 32))
    
    # convert to HSV
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
       
    # crop image and retain only H and S channels
    image = image[8:28,:,:2]
    
    # normalize
    image = image / 255. - 0.5
    image = np.array(image)
    
    return image

    
def X_generator(samples, batch_size=32):
    
    num_samples = len(samples)
    
    while 1: # Loop forever so the generator never terminates     
        
        shuffle(samples, random_state = 12)
        
        for offset in range(0, num_samples, batch_size):
            
            batch_samples = samples[offset:offset+batch_size]
            
            images = []
            angles = []
            
            for batch_sample in batch_samples:
                
                # Read image using the file path
                image = cv2.imread(batch_sample[0].strip())
                
                # Flip the image, if the tag is -1                
                if batch_sample[2] == -1:
                    image = cv2.flip(image, 1)
                    
                # Preprocess image
                image = image_preprocessing(image)
                                
                # Read the corresponding steering angle
                angle = batch_sample[1]
                
                images.append(image)
                angles.append(angle)

            X = np.array(images)
            y = np.array(angles)
            
            yield (X, y)
            
            
        
def LeNet_mod(inp_shape):

    model = Sequential()
    
    # Convolution layer of 6 filters of size (5, 5) followed by Maxpooling layer - same as LeNet architecture
    model.add(Conv2D(6, 5, 5, border_mode='valid',input_shape=inp_shape, activation='relu'))
    model.add(MaxPooling2D())
    
    # Additional dropout layer to prevent overfitting
    model.add(Dropout(0.25))    

    model.add(Flatten())

    # Fully connected layer with reduced size compared to LeNet
    model.add(Dense(4, activation='relu'))   

    model.add(Dense(1))
    
    return model
    
    
if __name__ == '__main__':
    
    # load training data
    path = 'data/driving_log.csv'
    samples, labels = load_data(path)
    
    # data augmentation
    augmented_samples = augment_data(samples, correction = 0.2)
    
    # shuffle and split the augmented data into training and validation sets
    shuffle(augmented_samples, random_state = 12)
    training_samples, validation_samples = train_test_split(augmented_samples, test_size=0.1, random_state = 12)
    
    # initialise the training and validation generators
    train_generator = X_generator(training_samples, batch_size=256)
    validation_generator = X_generator(validation_samples, batch_size=256)
    
    # initialise the model for training
    model = LeNet_mod((20, 64, 2))
    
    # compile and train the model
    model.compile(loss='mse', optimizer='adam')
    model.fit_generator(train_generator, samples_per_epoch= len(training_samples), validation_data=validation_generator, nb_val_samples=len(validation_samples), nb_epoch=20)
    
    # save the trained model        
    model.save("model.h5")
    print("Model saved.")
'''
@author: Shaiful Nizam
@since: December 14, 2021
@organization: RoboLab (Ritsumeikan University, College of Information Science and Engineering)
References:
    https://stackoverflow.com/questions/59935503/wrong-order-of-frames-creating-gif-animation-from-png-files
    https://www.codegrepper.com/code-examples/python/make+an+animation+with+pictures+python
'''
import glob
import re
from PIL import Image

def get_number(file_name):
    m = re.findall('FIG_([0-9]*)\.', file_name)[0]
    return int(m)

def make_gif(frame_folder, tDuration):
    # Get all images in a folder as a glob
    imgs = glob.glob(frame_folder+"/*.png")
    print(imgs)
    
    # The images needed to be sorted because glob returns the images in random order.
    # The GIF will only be animated correctly when images are ordered. That is why this is an important step.
    lst_numbers = [get_number(i) for i in imgs]
    print(lst_numbers)
    lst_number_files = sorted(list(zip(lst_numbers, imgs)))
    imgs_sorted = [i[1] for i in lst_number_files]
    print(imgs_sorted)

    # Begin working on the animated GIF.
    frames = []
    for i in imgs_sorted:
        new_frame = Image.open(i)
        frames.append(new_frame)
    
    # Save into a GIF file that loops forever
    frames[0].save(frame_folder+"/animated.gif", format='GIF',
                   append_images=frames[1:],
                   save_all=True,
                   duration=tDuration, loop=0)
    print("Job completed!")
    
if __name__ == "__main__":
    # make_gif("images/drone01", 100)
    # make_gif("images/drone03", 100)
    # make_gif("images/drone05", 100)
    # make_gif("images/drone07", 100)
    # make_gif("images/drone08", 100)
    make_gif("images/monitor", 100)
    make_gif("images/plot", 400)
    
    
import pandas as pd
from PIL import Image, ImageDraw
from PIL import ImageFont
import os

base_map = Image.open(r"d:\Bao Truong\Pictures\Location-of-the-Illinois-River-basin resized base.png").convert("RGBA")

b1x=204
b1y=812
b2x=200
b2y=637
b3x=340
b3y=523
b4x=433
b4y=379
b5x=608
b5y=169

region_coords = {
    "Region 1": [(b1x, b1y), (b1x-9, b1y-10), (b1x-2, b1y-20), (192, 746), (195, 720), (195, 712), (199,705), (201, 676), (206, 652), (219, 637), (226, 640), (215, 654), (210, 678), (205, 703), (211, 712), (202, 721), (199, 746), (201, 769), (199, 779), (203, 797), (207, 804)],
    "Region 2": [(b2x, b2y), (217, 625), (224, 623), (238, 606), (247, 591), (265, 580), (276, 559), (295, 545), (307, 530), (315, 528), (320, 520), (330, 519),
                  (340, 519), (335, 520), (325, 528), (313, 530), (310, 540), (298, 567), (271, 586), (260, 598), (253, 605), (230, 628), (220, 630), (216, 639)],
    "Region 3": [(b3x, b3y), (355, 489), (363, 462), (368, 456), (370, 461),  (373, 435), (388, 408), (387, 377), (399, 388), (433, 386),
                 (433, 376), (399, 378), (394, 397), (400, 408), (383, 435),(380, 461),  (378, 456), (373, 462), (365,489), (350, 523)],
    "Region 4": [(b4x, b4y), (466, 369), (493, 374), (514, 374), (544, 361), (557, 352), (563, 339), (566, 313), (638, 258),
                 (638, 268), (573, 317), (573, 339), (564, 357), (544, 371), (514, 384), (514, 384), (493, 384), (466, 379), (433, 389)],
    "Region 5": [(b5x, b5y), (601, 198), (627, 234), (636, 288), (656, 308),  (693, 321), (723, 321), (752, 304), (774, 259),
                 (764, 259), (748, 296), (723, 311), (696, 314), (656, 318), (641, 282), (632, 234), (606, 198), (613, 169)],
}

df = pd.read_csv(r"D:\Bao Truong\Downloads\D - Sheet1.csv")

os.makedirs("frames", exist_ok=True)
frames = []

for _, row in df.iterrows():
    year = row["Year"]
    img = base_map.copy()
    draw = ImageDraw.Draw(img, "RGBA")

    for region, coords in region_coords.items():
        if row[region] == "Y":
            draw.polygon(coords, fill=(255, 0, 0, 100))

            region_number = region.split()[-1]

            avg_x = sum(x for x, _ in coords) // len(coords)
            avg_y = sum(y for _, y in coords) // len(coords)

            label_x = avg_x + 30
            label_y = avg_y - 20

            region_font = ImageFont.truetype("arialbd.ttf", 30)
            draw.text((label_x, label_y), "Region " + region_number, font = region_font, fill = "black")


    font = ImageFont.truetype("arial.ttf", 32)
    text = f"Year: {year}"
    text_size = draw.textbbox((0,0), text, font=font)
    draw.rectangle([text_size[0] + 15, text_size[1] + 15, text_size[2] + 15, text_size[3] + 15], fill=(255, 255, 255, 200))
    draw.text((20, 20), text, font=font, fill="black")

    frame_path = f"frames/frame_{year}.png"
    img.save(frame_path)
    frames.append(img)

frames[0].save("illinois_river_animation.gif", save_all = True,
               append_images = frames[1:], duration = 800, loop = 0)
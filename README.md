# Texture-Transfer
## How to find the similar patch?
First of all, I sum up all the pixel difference in gray scale in a particular patch rectangle between texture and image.
Second, find the patch rectangle which has the smallest difference, and that's the most similar part.

***difference = |image.pixel(x,y) - texture.pixel(x,y)|***

## Here is some option setting
```
main.exe –i infile –o outfile –t texture_file –p patch_size -r overlap
```
- -i input file name
- -o output file name
- -t texture file name
- -p patch size (ex.32)
- -r overlap (ex.4)

## Example
### Here is the input file, texture_transfer.jpg
![texture_transfer](https://user-images.githubusercontent.com/24976415/26862834-dd0553c4-4b80-11e7-86b0-11486ec07428.jpg)

### It is the texture file, texture_rice.jpg
![texture_rice](https://user-images.githubusercontent.com/24976415/26862861-0f59acf8-4b81-11e7-823f-e4281243ebef.jpg)

### And it is the output it would look like
- Patch size : 16
- Overlap : 4

![texture_transfer_out](https://user-images.githubusercontent.com/24976415/26862879-2d0ca3ea-4b81-11e7-84bb-ec89f1f4b9c0.jpg)


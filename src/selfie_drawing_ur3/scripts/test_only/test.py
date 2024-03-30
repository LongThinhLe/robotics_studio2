# import tkinter as tk
# from tkinter import ttk, filedialog
# import cv2
# from PIL import Image, ImageTk
# import os

# class SelfieDrawingApp:
#     def __init__(self, master):
#         self.master = master
#         master.title("Automated Artistic Portraiture")

#         # Create a notebook (tabbed interface)
#         self.notebook = ttk.Notebook(master)
#         self.notebook.pack(fill='both', expand=True)

#         # Create the tabs
#         self.tab_take_picture = ttk.Frame(self.notebook)
#         self.tab_generate_greyscale = ttk.Frame(self.notebook)
#         self.tab_generate_gcode = ttk.Frame(self.notebook)
#         self.tab_robot_draw = ttk.Frame(self.notebook)

#         # Add tabs to the notebook
#         self.notebook.add(self.tab_take_picture, text="Take Picture")
#         self.notebook.add(self.tab_generate_greyscale, text="Generate Greyscale Image")
#         self.notebook.add(self.tab_generate_gcode, text="Generate Gcode")
#         self.notebook.add(self.tab_robot_draw, text="Robot Draw")

#         # Initialize components for the "Take Picture" tab
#         self.init_take_picture_tab()

#     def init_take_picture_tab(self):
#         # Create a frame to hold the preview screen and its label
#         preview_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
#         preview_frame.pack(side=tk.LEFT, padx=10, pady=10)

#         # Define the desired height for the screens
#         screen_height = 480  # Adjusted to 480 pixels

#         # Create a canvas for the preview screen
#         self.canvas_preview = tk.Canvas(preview_frame, width=640, height=screen_height)
#         self.canvas_preview.pack()

#         # Create a label for the preview screen with increased font size
#         lbl_preview = tk.Label(preview_frame, text="Preview Screen", font=("Arial", 20))
#         lbl_preview.pack()

#         # Create a frame to hold the capture screen and its label
#         capture_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
#         capture_frame.pack(side=tk.LEFT, padx=10, pady=10)

#         # Create a canvas for the captured picture screen
#         self.canvas_capture = tk.Canvas(capture_frame, width=640, height=screen_height)
#         self.canvas_capture.pack()

#         # Create a label for the capture screen with increased font size
#         lbl_capture = tk.Label(capture_frame, text="Capture Screen", font=("Arial", 20))
#         lbl_capture.pack()

#         # Create a frame to hold the buttons
#         button_frame = tk.Frame(self.tab_take_picture)
#         button_frame.pack(side=tk.RIGHT, padx=10, pady=10, fill='y')  # Fill the available vertical space

#         # Create the buttons for taking a picture and resetting
#         btn_capture = tk.Button(button_frame, text="Take Picture", command=self.take_picture, width=20)  # Set width to fill available horizontal space
#         btn_reset = tk.Button(button_frame, text="Reset", command=self.reset, width=20)  # Set width to fill available horizontal space
#         btn_capture.pack(side=tk.TOP, padx=10, pady=5, fill='x')  # Fill the available horizontal space
#         btn_reset.pack(side=tk.TOP, padx=10, pady=5, fill='x')  # Fill the available horizontal space

#         # Initialize captured photo variable
#         self.photo = None

#         # Start the webcam preview
#         self.start_preview()

#     def start_preview(self):
#         # Access the webcam
#         self.cap = cv2.VideoCapture(0)

#         # Display the live feed in the preview canvas
#         self.update_preview()

#     def update_preview(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the preview canvas
#             screen_height = self.canvas_preview.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the preview canvas with the new frame
#             self.canvas_preview.create_image(0, 0, anchor=tk.NW, image=photo)
#             self.canvas_preview.image = photo

#         # Schedule the next update
#         self.canvas_preview.after(10, self.update_preview)

#     def take_picture(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the capture canvas
#             screen_height = self.canvas_capture.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             self.photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the capture canvas with the captured picture
#             self.canvas_capture.create_image(0, 0, anchor=tk.NW, image=self.photo)
#             self.canvas_capture.image = self.photo

#             # Save the captured picture to a file
#             file_path = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("PNG files", "*.png"), ("All files", "*.*")])
#             if file_path:
#                 img = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)  # Convert back to BGR for cv2
#                 cv2.imwrite(file_path, img)
#                 print("Picture saved:", file_path)

#     def reset(self):
#         # Clear the captured photo canvas
#         self.canvas_capture.delete("all")
#         self.photo = None

#     def __del__(self):
#         # Release the webcam when the application is closed
#         if hasattr(self, 'cap'):
#             self.cap.release()

# def main():
#     root = tk.Tk()
#     app = SelfieDrawingApp(root)
#     root.mainloop()

# if __name__ == "__main__":
#     main()







# import tkinter as tk
# from tkinter import ttk
# import cv2
# from PIL import Image, ImageTk

# class SelfieDrawingApp:
#     def __init__(self, master):
#         self.master = master
#         master.title("Automated Artistic Portraiture")

#         # Create a notebook (tabbed interface)
#         self.notebook = ttk.Notebook(master)
#         self.notebook.pack(fill='both', expand=True)

#         # Create the tabs
#         self.tab_take_picture = ttk.Frame(self.notebook)
#         self.tab_generate_greyscale = ttk.Frame(self.notebook)
#         self.tab_generate_gcode = ttk.Frame(self.notebook)
#         self.tab_robot_draw = ttk.Frame(self.notebook)

#         # Add tabs to the notebook
#         self.notebook.add(self.tab_take_picture, text="Take Picture")
#         self.notebook.add(self.tab_generate_greyscale, text="Generate Greyscale Image")
#         self.notebook.add(self.tab_generate_gcode, text="Generate Gcode")
#         self.notebook.add(self.tab_robot_draw, text="Robot Draw")

#         # Initialize components for the "Take Picture" tab
#         self.init_take_picture_tab()

#     def init_take_picture_tab(self):
#         # Create a frame to hold the preview screen and its label
#         preview_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
#         preview_frame.pack(side=tk.LEFT, padx=10, pady=10)

#         # Define the desired height for the screens
#         screen_height = 480  # Adjusted to 480 pixels

#         # Create a canvas for the preview screen
#         self.canvas_preview = tk.Canvas(preview_frame, width=640, height=screen_height)
#         self.canvas_preview.pack()

#         # Create a label for the preview screen with increased font size
#         lbl_preview = tk.Label(preview_frame, text="Preview Screen", font=("Arial", 20))
#         lbl_preview.pack()

#         # Create a frame to hold the capture screen and its label
#         capture_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
#         capture_frame.pack(side=tk.LEFT, padx=10, pady=10)

#         # Create a canvas for the captured picture screen
#         self.canvas_capture = tk.Canvas(capture_frame, width=640, height=screen_height)
#         self.canvas_capture.pack()

#         # Create a label for the capture screen with increased font size
#         lbl_capture = tk.Label(capture_frame, text="Capture Screen", font=("Arial", 20))
#         lbl_capture.pack()

#         # Create a frame to hold the buttons
#         button_frame = tk.Frame(self.tab_take_picture)
#         button_frame.pack(side=tk.RIGHT, padx=10, pady=10, fill='y')  # Fill the available vertical space

#         # Create the buttons for taking a picture and resetting
#         btn_capture = tk.Button(button_frame, text="Take Picture", command=self.take_picture, width=20)  # Set width to fill available horizontal space
#         btn_reset = tk.Button(button_frame, text="Reset", command=self.reset, width=20)  # Set width to fill available horizontal space
#         btn_capture.pack(side=tk.TOP, padx=10, pady=5, fill='x')  # Fill the available horizontal space
#         btn_reset.pack(side=tk.TOP, padx=10, pady=5, fill='x')  # Fill the available horizontal space

#         # Initialize captured photo variable
#         self.photo = None

#         # Start the webcam preview
#         self.start_preview()

#     def start_preview(self):
#         # Access the webcam
#         self.cap = cv2.VideoCapture(0)

#         # Display the live feed in the preview canvas
#         self.update_preview()

#     def update_preview(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the preview canvas
#             screen_height = self.canvas_preview.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the preview canvas with the new frame
#             self.canvas_preview.create_image(0, 0, anchor=tk.NW, image=photo)
#             self.canvas_preview.image = photo

#         # Schedule the next update
#         self.canvas_preview.after(10, self.update_preview)

#     def take_picture(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the capture canvas
#             screen_height = self.canvas_capture.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             self.photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the capture canvas with the captured picture
#             self.canvas_capture.create_image(0, 0, anchor=tk.NW, image=self.photo)
#             self.canvas_capture.image = self.photo

#     def reset(self):
#         # Clear the captured photo canvas
#         self.canvas_capture.delete("all")
#         self.photo = None

#     def __del__(self):
#         # Release the webcam when the application is closed
#         if hasattr(self, 'cap'):
#             self.cap.release()

# def main():
#     root = tk.Tk()
#     app = SelfieDrawingApp(root)
#     root.mainloop()

# if __name__ == "__main__":
#     main()









# import tkinter as tk
# from tkinter import ttk
# import cv2
# from PIL import Image, ImageTk

# class SelfieDrawingApp:
#     def __init__(self, master):
#         self.master = master
#         master.title("Automated Artistic Portraiture")

#         # Create a notebook (tabbed interface)
#         self.notebook = ttk.Notebook(master)
#         self.notebook.pack(fill='both', expand=True)

#         # Create the tabs
#         self.tab_take_picture = ttk.Frame(self.notebook)
#         self.tab_generate_greyscale = ttk.Frame(self.notebook)
#         self.tab_generate_gcode = ttk.Frame(self.notebook)
#         self.tab_robot_draw = ttk.Frame(self.notebook)

#         # Add tabs to the notebook
#         self.notebook.add(self.tab_take_picture, text="Take Picture")
#         self.notebook.add(self.tab_generate_greyscale, text="Generate Greyscale Image")
#         self.notebook.add(self.tab_generate_gcode, text="Generate Gcode")
#         self.notebook.add(self.tab_robot_draw, text="Robot Draw")

#         # Initialize components for the "Take Picture" tab
#         self.init_take_picture_tab()

#     def init_take_picture_tab(self):
#         # Create a frame to hold the preview screen and its label
#         preview_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
#         preview_frame.pack(side=tk.LEFT, padx=10, pady=10)

#         # Define the desired height for the screens
#         screen_height = 480  # Adjusted to 480 pixels

#         # Create a canvas for the preview screen
#         self.canvas_preview = tk.Canvas(preview_frame, width=640, height=screen_height)
#         self.canvas_preview.pack()

#         # Create a label for the preview screen with increased font size
#         lbl_preview = tk.Label(preview_frame, text="Preview Screen", font=("Arial", 20))
#         lbl_preview.pack()

#         # Create a frame to hold the capture screen and its label
#         capture_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
#         capture_frame.pack(side=tk.LEFT, padx=10, pady=10)

#         # Create a canvas for the captured picture screen
#         self.canvas_capture = tk.Canvas(capture_frame, width=640, height=screen_height)
#         self.canvas_capture.pack()

#         # Create a label for the capture screen with increased font size
#         lbl_capture = tk.Label(capture_frame, text="Capture Screen", font=("Arial", 20))
#         lbl_capture.pack()

#         # Create a frame to hold the buttons
#         button_frame = tk.Frame(self.tab_take_picture)
#         button_frame.pack(side=tk.RIGHT, padx=10, pady=10, fill= 'y')

#         # Create the buttons for taking a picture and resetting
#         btn_capture = tk.Button(button_frame, text="Take Picture", command=self.take_picture)
#         btn_reset = tk.Button(button_frame, text="Reset", command=self.reset)
#         btn_capture.pack(side=tk.TOP, padx=10, pady=5)
#         btn_reset.pack(side=tk.TOP, padx=10, pady=5)

#         # Initialize captured photo variable
#         self.photo = None

#         # Start the webcam preview
#         self.start_preview()

#     def start_preview(self):
#         # Access the webcam
#         self.cap = cv2.VideoCapture(0)

#         # Display the live feed in the preview canvas
#         self.update_preview()

#     def update_preview(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the preview canvas
#             screen_height = self.canvas_preview.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the preview canvas with the new frame
#             self.canvas_preview.create_image(0, 0, anchor=tk.NW, image=photo)
#             self.canvas_preview.image = photo

#         # Schedule the next update
#         self.canvas_preview.after(10, self.update_preview)

#     def take_picture(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the capture canvas
#             screen_height = self.canvas_capture.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             self.photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the capture canvas with the captured picture
#             self.canvas_capture.create_image(0, 0, anchor=tk.NW, image=self.photo)
#             self.canvas_capture.image = self.photo

#     def reset(self):
#         # Clear the captured photo canvas
#         self.canvas_capture.delete("all")
#         self.photo = None

#     def __del__(self):
#         # Release the webcam when the application is closed
#         if hasattr(self, 'cap'):
#             self.cap.release()

# def main():
#     root = tk.Tk()
#     app = SelfieDrawingApp(root)
#     root.mainloop()

# if __name__ == "__main__":
#     main()


# import tkinter as tk
# import cv2
# from PIL import Image, ImageTk

# class SelfieDrawingApp:
#     def __init__(self, master):
#         self.master = master
#         master.title("Automated Artistic Portraiture")
#         master.resizable(False, False)  # Make the window unresizeable

#         # Create a frame to hold the preview screen and its label
#         self.preview_frame = tk.Frame(master, bd=2, relief=tk.SOLID)
#         self.preview_frame.pack(side=tk.LEFT, padx=10, pady=10)

#         # Define the desired height for the screens
#         screen_height = 480  # Adjusted to 480 pixels

#         # Create a canvas for the preview screen
#         self.canvas_preview = tk.Canvas(self.preview_frame, width=640, height=screen_height)
#         self.canvas_preview.pack()

#         # Create a label for the preview screen with increased font size
#         self.lbl_preview = tk.Label(self.preview_frame, text="Preview Screen", font=("Arial", 20))
#         self.lbl_preview.pack()

#         # Create a frame to hold the capture screen and its label
#         self.capture_frame = tk.Frame(master, bd=2, relief=tk.SOLID)
#         self.capture_frame.pack(side=tk.LEFT, padx=10, pady=10)

#         # Create a canvas for the captured picture screen
#         self.canvas_capture = tk.Canvas(self.capture_frame, width=640, height=screen_height)
#         self.canvas_capture.pack()

#         # Create a label for the capture screen with increased font size
#         self.lbl_capture = tk.Label(self.capture_frame, text="Capture Screen", font=("Arial", 20))
#         self.lbl_capture.pack()

#         # Create a frame to hold the buttons
#         self.button_frame = tk.Frame(master)
#         self.button_frame.pack(side=tk.RIGHT, padx=10, pady=10)

#         # Create the buttons for taking a picture and resetting
#         self.btn_capture = tk.Button(self.button_frame, text="Take Picture", command=self.take_picture)
#         self.btn_reset = tk.Button(self.button_frame, text="Reset", command=self.reset)
#         self.btn_capture.pack(side=tk.TOP, padx=10, pady=5)
#         self.btn_reset.pack(side=tk.TOP, padx=10, pady=5)

#         # Start the webcam preview
#         self.start_preview()

#         # Initialize captured photo variable
#         self.photo = None

#     def start_preview(self):
#         # Access the webcam
#         self.cap = cv2.VideoCapture(0)

#         # Display the live feed in the preview canvas
#         self.update_preview()

#     def update_preview(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the preview canvas
#             screen_height = self.canvas_preview.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the preview canvas with the new frame
#             self.canvas_preview.create_image(0, 0, anchor=tk.NW, image=photo)
#             self.canvas_preview.image = photo

#         # Schedule the next update
#         self.canvas_preview.after(10, self.update_preview)

#     def take_picture(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the capture canvas
#             screen_height = self.canvas_capture.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             self.photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the capture canvas with the captured picture
#             self.canvas_capture.create_image(0, 0, anchor=tk.NW, image=self.photo)
#             self.canvas_capture.image = self.photo

#     def reset(self):
#         # Clear the captured photo canvas
#         self.canvas_capture.delete("all")
#         self.photo = None

#     def __del__(self):
#         # Release the webcam when the application is closed
#         if hasattr(self, 'cap'):
#             self.cap.release()


# def main():
#     root = tk.Tk()
#     app = SelfieDrawingApp(root)
#     root.mainloop()

# if __name__ == "__main__":
#     main()

















# import tkinter as tk
# import cv2
# from PIL import Image, ImageTk

# class SelfieDrawingApp:
#     def __init__(self, master):
#         self.master = master
#         master.title("Automated Artistic Portraiture")

#         # Create a frame to hold the preview screen and its label
#         self.preview_frame = tk.Frame(master, bd=2, relief=tk.SOLID)
#         self.preview_frame.pack(side=tk.LEFT, padx=10, pady=10)

#         # Define the desired height for the screens
#         screen_height = 480  # Adjusted to 480 pixels

#         # Create a canvas for the preview screen
#         self.canvas_preview = tk.Canvas(self.preview_frame, width=640, height=screen_height)
#         self.canvas_preview.pack()

#         # Create a label for the preview screen
#         self.lbl_preview = tk.Label(self.preview_frame, text="Preview Screen")
#         self.lbl_preview.pack()

#         # Create a frame to hold the capture screen and its label
#         self.capture_frame = tk.Frame(master, bd=2, relief=tk.SOLID)
#         self.capture_frame.pack(side=tk.LEFT, padx=10, pady=10)

#         # Create a canvas for the captured picture screen
#         self.canvas_capture = tk.Canvas(self.capture_frame, width=640, height=screen_height)
#         self.canvas_capture.pack()

#         # Create a label for the capture screen
#         self.lbl_capture = tk.Label(self.capture_frame, text="Capture Screen")
#         self.lbl_capture.pack()

#         # Create a frame to hold the buttons
#         self.button_frame = tk.Frame(master)
#         self.button_frame.pack(side=tk.RIGHT, padx=10, pady=10)

#         # Create the buttons for taking a picture and resetting
#         self.btn_capture = tk.Button(self.button_frame, text="Take Picture", command=self.take_picture)
#         self.btn_reset = tk.Button(self.button_frame, text="Reset", command=self.reset)
#         self.btn_capture.pack(side=tk.TOP, padx=10, pady=5)
#         self.btn_reset.pack(side=tk.TOP, padx=10, pady=5)

#         # Start the webcam preview
#         self.start_preview()

#         # Initialize captured photo variable
#         self.photo = None

#     def start_preview(self):
#         # Access the webcam
#         self.cap = cv2.VideoCapture(0)

#         # Display the live feed in the preview canvas
#         self.update_preview()

#     def update_preview(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the preview canvas
#             screen_height = self.canvas_preview.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the preview canvas with the new frame
#             self.canvas_preview.create_image(0, 0, anchor=tk.NW, image=photo)
#             self.canvas_preview.image = photo

#         # Schedule the next update
#         self.canvas_preview.after(10, self.update_preview)

#     def take_picture(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the capture canvas
#             screen_height = self.canvas_capture.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             self.photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the capture canvas with the captured picture
#             self.canvas_capture.create_image(0, 0, anchor=tk.NW, image=self.photo)
#             self.canvas_capture.image = self.photo

#     def reset(self):
#         # Clear the captured photo canvas
#         self.canvas_capture.delete("all")
#         self.photo = None

#     def __del__(self):
#         # Release the webcam when the application is closed
#         if hasattr(self, 'cap'):
#             self.cap.release()


# def main():
#     root = tk.Tk()
#     app = SelfieDrawingApp(root)
#     root.mainloop()

# if __name__ == "__main__":
#     main()





# import tkinter as tk
# import cv2
# from PIL import Image, ImageTk

# class SelfieDrawingApp:
#     def __init__(self, master):
#         self.master = master
#         master.title("Automated Artistic Portraiture")

#         # Create a frame to hold the preview screen and its label
#         self.preview_frame = tk.Frame(master)
#         self.preview_frame.pack(side=tk.LEFT, padx=10)

#         # Define the desired height for the screens
#         screen_height = 480  # Adjusted to 480 pixels

#         # Create a canvas for the preview screen
#         self.canvas_preview = tk.Canvas(self.preview_frame, width=640, height=screen_height)
#         self.canvas_preview.pack()

#         # Create a label for the preview screen
#         self.lbl_preview = tk.Label(self.preview_frame, text="Preview Screen")
#         self.lbl_preview.pack()

#         # Create a frame to hold the capture screen and its label
#         self.capture_frame = tk.Frame(master)
#         self.capture_frame.pack(side=tk.RIGHT, padx=10)

#         # Create a canvas for the captured picture screen
#         self.canvas_capture = tk.Canvas(self.capture_frame, width=640, height=screen_height)
#         self.canvas_capture.pack()

#         # Create a label for the capture screen
#         self.lbl_capture = tk.Label(self.capture_frame, text="Capture Screen")
#         self.lbl_capture.pack()

#         # Create the buttons for taking a picture and resetting
#         self.btn_capture = tk.Button(master, text="Take Picture", command=self.take_picture)
#         self.btn_reset = tk.Button(master, text="Reset", command=self.reset)
#         self.btn_capture.pack(side=tk.LEFT, padx=10)
#         self.btn_reset.pack(side=tk.RIGHT, padx=10)

#         # Start the webcam preview
#         self.start_preview()

#         # Initialize captured photo variable
#         self.photo = None

#     def start_preview(self):
#         # Access the webcam
#         self.cap = cv2.VideoCapture(0)

#         # Display the live feed in the preview canvas
#         self.update_preview()

#     def update_preview(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the preview canvas
#             screen_height = self.canvas_preview.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the preview canvas with the new frame
#             self.canvas_preview.create_image(0, 0, anchor=tk.NW, image=photo)
#             self.canvas_preview.image = photo

#         # Schedule the next update
#         self.canvas_preview.after(10, self.update_preview)

#     def take_picture(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the capture canvas
#             screen_height = self.canvas_capture.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             self.photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the capture canvas with the captured picture
#             self.canvas_capture.create_image(0, 0, anchor=tk.NW, image=self.photo)
#             self.canvas_capture.image = self.photo

#     def reset(self):
#         # Clear the captured photo canvas
#         self.canvas_capture.delete("all")
#         self.photo = None

#     def __del__(self):
#         # Release the webcam when the application is closed
#         if hasattr(self, 'cap'):
#             self.cap.release()


# def main():
#     root = tk.Tk()
#     app = SelfieDrawingApp(root)
#     root.mainloop()

# if __name__ == "__main__":
#     main()





# import tkinter as tk
# import cv2
# from PIL import Image, ImageTk

# class SelfieDrawingApp:
#     def __init__(self, master):
#         self.master = master
#         master.title("Automated Artistic Portraiture")

#         # Create a frame to hold the canvases and the buttons
#         self.frame = tk.Frame(master)
#         self.frame.pack()

#         # Define the desired height for the screens
#         screen_height = 480  # You can adjust this value as needed

#         # Create a canvas for the preview screen
#         self.canvas_preview = tk.Canvas(self.frame, width=640, height=screen_height)
#         self.canvas_preview.pack(side=tk.LEFT, padx=10)

#         # Create a canvas for the captured picture screen
#         self.canvas_capture = tk.Canvas(self.frame, width=640, height=screen_height)
#         self.canvas_capture.pack(side=tk.RIGHT, padx=10)

#         # Create the buttons for taking a picture and resetting
#         self.btn_capture = tk.Button(master, text="Take Picture", command=self.take_picture)
#         # self.btn_reset = tk.Button(master, text="Reset", command=self.reset)
#         self.btn_capture.pack(side=tk.LEFT, padx=10)
#         # self.btn_reset.pack(side=tk.RIGHT, padx=10)

#         # Start the webcam preview
#         self.start_preview()

#         # Initialize captured photo variable
#         self.photo = None

#     def start_preview(self):
#         # Access the webcam
#         self.cap = cv2.VideoCapture(0)

#         # Display the live feed in the preview canvas
#         self.update_preview()

#     def update_preview(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the preview canvas
#             screen_height = self.canvas_preview.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the preview canvas with the new frame
#             self.canvas_preview.create_image(0, 0, anchor=tk.NW, image=photo)
#             self.canvas_preview.image = photo

#         # Schedule the next update
#         self.canvas_preview.after(10, self.update_preview)

#     def take_picture(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the capture canvas
#             screen_height = self.canvas_preview.winfo_height()
#             frame_resized = cv2.resize(frame_rgb, (640, screen_height))

#             # Convert the frame to ImageTk format
#             self.photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the capture canvas with the captured picture
#             self.canvas_capture.create_image(0, 0, anchor=tk.NW, image=self.photo)
#             self.canvas_capture.image = self.photo

#     # def reset(self):
#     #     # Clear the captured photo canvas
#     #     self.canvas_capture.delete("all")
#     #     self.photo = None

#     def __del__(self):
#         # Release the webcam when the application is closed
#         if hasattr(self, 'cap'):
#             self.cap.release()


# def main():
#     root = tk.Tk()
#     app = SelfieDrawingApp(root)
#     root.mainloop()

# if __name__ == "__main__":
#     main()



# import tkinter as tk
# import cv2
# from PIL import Image, ImageTk

# class SelfieDrawingApp:
#     def __init__(self, master):
#         self.master = master
#         master.title("Automated Artistic Portraiture")

#         # Create a frame to hold the canvas and the button
#         self.frame = tk.Frame(master)
#         self.frame.pack()

#         # Create a canvas for the preview screen
#         self.canvas_preview = tk.Canvas(self.frame, width=640, height=360)
#         self.canvas_preview.pack(side=tk.LEFT, padx=10)

#         # Create a canvas for the captured picture screen
#         self.canvas_capture = tk.Canvas(self.frame, width=640, height=360)
#         self.canvas_capture.pack(side=tk.RIGHT, padx=10)

#         # Create the button for taking a picture
#         self.btn_capture = tk.Button(master, text="Take Picture", command=self.take_picture)
#         self.btn_capture.pack()

#         # Start the webcam preview
#         self.start_preview()

#     def start_preview(self):
#         # Access the webcam
#         self.cap = cv2.VideoCapture(0)

#         # Display the live feed in the preview canvas
#         self.update_preview()

#     def update_preview(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the preview canvas
#             frame_resized = cv2.resize(frame_rgb, (640, 360))

#             # Convert the frame to ImageTk format
#             photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the preview canvas with the new frame
#             self.canvas_preview.create_image(0, 0, anchor=tk.NW, image=photo)
#             self.canvas_preview.image = photo

#         # Schedule the next update
#         self.canvas_preview.after(10, self.update_preview)

#     def take_picture(self):
#         # Capture a frame
#         ret, frame = self.cap.read()

#         if ret:
#             # Convert the frame from BGR to RGB
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#             # Resize the frame to fit the capture canvas
#             frame_resized = cv2.resize(frame_rgb, (640, 360))

#             # Convert the frame to ImageTk format
#             photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

#             # Update the capture canvas with the captured picture
#             self.canvas_capture.create_image(0, 0, anchor=tk.NW, image=photo)
#             self.canvas_capture.image = photo

#         # Close the webcam
#         self.cap.release()


# def main():
#     root = tk.Tk()
#     app = SelfieDrawingApp(root)
#     root.mainloop()

# if __name__ == "__main__":
#     main()

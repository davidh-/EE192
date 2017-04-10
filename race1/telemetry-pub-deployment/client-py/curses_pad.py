import curses
import time
import sys

MAX_CONSOLE_LINES = 10
#maximum number of lines to show on the console.


stdscr = None
console_lines = ["" for _ in range(MAX_CONSOLE_LINES)]
input_line = ""
line_to_send = None #the line to send.  Originally None, this code sets it to some string.  Other code will have to set it back to none.

height, width = 10,10 #just initial guesses.  They'll be overwritten first time show is called.

last_recieved = time.time()
packet_timeout = .1 #100ms between explicitly saying i've had bad packets.  Any issues less than that timeframe long will just get considered as the same packet.

incoming_byte_stream_to_console = ""

def run():
    #usually used for testing purposes.  The while true has to be located elsewhere if there's more code that needs a while true.
    while True:
    
        user_input()     
        show()
        #loop_ct += 1

def init(stdscr_in):
    global stdscr
    stdscr = stdscr_in
    stdscr.nodelay(1)

def init_run(stdscr_in):
    init(stdscr_in)
    run()


def user_input():
    global stdscr #get the global curses instance.
    global send_user_in
    global line_to_send 
    #get lines in.  Update handles displaying approprpriately.

    while True:
      c = stdscr.getch()
      if c < 0:
        #i ran out of characters to get
        break
      elif 32 <= c <= 127:
        #append the human-readable string.  Not sure if i should inlucde 127 (delete), but nikita does.
        add_input_char(chr(c))
      elif c == 10:
        #user hits 'return'
        line_to_send = input_line
        #
        clear_input_line()
      elif c == 263: #apparently backspace is 263 in curses land, not 8
        delete_input_char()
        #user just hit 'delete'
    #stdscr.refresh()
    stdscr.noutrefresh() 


def show_input_line(text):
    #update input line to read "text"
    global height
    global width


    input_win = curses.newwin(1, width, height - 1, 0)
    input_win.addstr(0, 0, ">>> " + text)
    input_win.refresh()


def is_ascii(s):
    return all(ord(c) < 128 for c in s)



def show_console(lines):
    #show 'lines' on the console
    global width
    global height

    console_pad = curses.newpad(100, width)

    for i, line in enumerate(lines[-(MAX_CONSOLE_LINES-1):]):
        #print i, line
        #print(i)
        #print(line.strip())
        if is_ascii(line): #filter out all the backslashes, that's most likely some binary stuff that you don't want to print
            try:
                console_pad.addstr(i+1, 0, line.strip())
            except:
                e = sys.exc_info()[0]
                add_console_line("Error In print: " + str(e))
                #we should probably remove the offending line
                console_lines[i] = ""
                #print("Error was " + str(e))
        else:
            line = "Not Ascii"
            console_pad.addstr(i+1, 0, line.strip())



    #fill the console with junk for testing
    #for y in range(0, 99):
    #    for x in range(0, width-1):
    #        console_pad.addch(y,x, ord('a') + (x*x+y*y) % 26)

    y1 = 0 #top of screen
    x1 = 0 #left of the screen
    y2 = height -1 #so we don't run off the end
    x2 = width -1 #to all the way on the right, minus 1
    
    console_pad.noutrefresh(0,0, y1,x1,y2,x2)


def add_console_line(newline):
    #add newline to the currently existing console output
    #does not handle showing operations
    global console_lines
    console_lines.append(newline)
    console_lines = console_lines[-MAX_CONSOLE_LINES:]

def append_console_line(newChar):
    #append newChar to the string that's being built for console line.  correctly handle \n and \r
    #somewhere, I want that if this line hasn't been updated within some timeout, just pring the thing.
    global console_lines
    global last_recieved
    global incoming_byte_stream_to_console

    last_recieved = time.time()
    #print("newchar is " + str(newChar) + '\n')

    if newChar == '\n':
        #ok, it's time to add the line!
        add_console_line(incoming_byte_stream_to_console)
        incoming_byte_stream_to_console = ""
    elif newChar == '\r':
        #yup, do nothing.
        pass
    elif (ord(newChar) < 32) or (ord(newChar) > 126):
        pass
        #do nothing.  These characters aren't human-legible.
    else:
        incoming_byte_stream_to_console += newChar

        '''
        new_line = console_lines[-1] + newChar
        if is_ascii(new_line):
            #ok, it's good.  Add it.
            console_lines[-1] = new_line
        else:
            #bad line, if i added it curses would crash.  I can't. 
            if (time.time() - last_corrupted) > packet_timeout:
                #ok i have a new bad packet.
                console_lines[-1] = "corrupted line"
                add_console_line("")
            else:
                console_lines[-1] = "" #just silently kill the packet.
            last_corrupted = time.time()
        '''

        #console_lines[-1] += newChar #append new char to console lines.  I tested it, this part works
        #making sure that console_lines don't include nasty code.
        #if not is_ascii(console_lines[-1]):
            #don't add it.
        #    console_lines[-1] = "corrupted line"
        #    add_console_line("")

    #toDo: how do we make sure that console lines wraps??  How do I deal with newlines?

def add_input_char(newChar):
    global input_line
    input_line = input_line + newChar
    #add input_character to the currently existing (and growing) line on the bottom of the screen.

def delete_input_char():
    #user just hit backspace key, let's delete the line.
    global input_line
    input_line = input_line[:-1]


def clear_input_line():
    global console_lines
    global input_line
    add_console_line(input_line)
    input_line = ""
    #input line gets added to console area, cleared.

def show():

    global height
    global width
    global MAX_CONSOLE_LINES
    global console_lines
    global incoming_byte_stream_to_console

    new_height, new_width = stdscr.getmaxyx()    #do this so screen will dynamically resise, if I do my job right.


    #and the size of my console also dynamically resises
    if new_height > height:
        #need to append new lines to console_lines, at the front.
        for i in range(new_height - height):
            console_lines.insert(0,"")
        MAX_CONSOLE_LINES = new_height - 1
    elif new_height < height:
        #I shrunk, need to trim console_lines
        MAX_CONSOLE_LINES = new_height - 1
        console_lines = console_lines[-MAX_CONSOLE_LINES:]
    height = new_height
    width = new_width


    if (time.time() - last_recieved) > packet_timeout:
        if len(incoming_byte_stream_to_console) > 0:
            #and make sure to check that i even have anything to print
            #it's time to print the incoming byte stream, i'm not gonna get anymore.
            add_console_line(incoming_byte_stream_to_console)
            incoming_byte_stream_to_console = ""


    #2 parts currently: input line, and the console output.  Screen only gets updated 
    show_input_line(input_line)
    show_console(console_lines)



if __name__ == "__main__":
    print("Hello World")
    curses.wrapper(lambda stdscr: init_run(stdscr))#start up user_input


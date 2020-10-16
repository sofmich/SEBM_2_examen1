# SEBM_2_examen1
Two sending tasks gets blocked when a third one is taking the text from buffer

Create a main tasks which creates the others three.

First and second task write a message on the same buffer... if the buffer is being used there won't be any chance for a task to write on it.

Once a task has written, the third one will get the message from the queue and print it to console.

Configurations for this project are for board kinetis k66F using FreeRTOS operating system and UART debug console.

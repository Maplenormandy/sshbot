BotClient
=========

Data visualizer and communicator for MASLAB

Initial Framework Idea:
There are 2 basic types of servers. There is the robot comms server
which directly facilitates communication with MASLAB bots. It listens on
an arbitrary socket. No more than 2 bots may be connected (enforced
through a lax check that can be alleviated in the future).

Then there is the relay server. An arbitrary amount of these can be
spawned, each with its own socket. Each relay protocol can attach to a
specific robot and recieve its data. From here it can go to a screen or
something...

The idea is that there is 1 comms server running and n relay servers. I
was thinking that each relay server/client corresponds to a single
"dashboard". In this way there will be 2 independant "dashboards", each
interfacing 1 robot, running during the competition. Of course, if
future competitions involve >2 robots on field this design will still
work.

how it actually works:
On the comms server there will be a database indicating users and their
corresponding tokens. This is because we don't want teams to impersonate
eachother without having to try really hard. At the moment this database
is simple a json file because I don't know which databases are good and
bad.

When a team sends data to the comms server it will be in json and
contain: {"token":"tokenvaluehere", "Field1":"value1", "Field2":"value2"...}
You can create this in python trivially by running a dictionary through
the built-in json module. The comms server will first make sure that the
token maps to a valid user (if it doesn't it sends a reprimand to the
attacker). After doing this, the server parses the message. It takes all
of the Field:value pairs, prefixes them with the username, and puts them 
into the multithreaded command queue.

Now when a relay server requests the next command it will search
through the queue for the first command to match the attached user and
return it to its client. Now the client should display it.


Heres a layout of sorts:
(This thing is screwed up on github. Sorry)
benbot    ---> |both send their commands to| 
alyssabot ---> |robot comms servers        | ----> parses and puts data in a multiprocess queue -->-|
                                                                                                    |
|<--------------------------------------------------------<------------------------------------------
|-> relayclientA can specify it 
    wants to recieve ben's transmissions ---> everytime relayclientA
                                              sends "n" the next item in
                                              the queue that came from
                                              ben is returned. Else
                                              'none'
A different relayclient (or the same one) can specify that it wants to
receive alyssabot's communication.


#UPDATE LOG

12/22/13 Added broadcast support as well as sysadmin support

11/29/13 Added websocket support. Works exactly the same way as the old
         botrelay protocol except now you can do it in javascript!

1/1/14   Basically done. Botview is working with streaming images. There
         is an admin protocol whose instructions are sent to every robot
         and displayed on the screen in a special admin field.
         Server->client communication also works. The botclient java
         helper api is still pending.

#! /usr/bin/env python

import rospy
import smach
from Move import Move
from Search import Search

"""
This class runs through the entire gate manuever.
          0 _____
           X_____\
   .-^-.  ||_| |_||  .-^-.
  /_\_/_\_|  |_|  |_/_\_/_\
  ||(_)| __\_____/__ |(_)||
  \/| | |::|\```/|::| | |\/
  /`---_|::|-+-+-|::|_---'\
 / /  \ |::|-|-|-|::| /  \ \
/_/   /|`--'-+-+-`--'|\   \_\
| \  / |===/_\ /_\===| \  / |
|  \/  /---/-/-\-\  o\  \/  |
| ||| | O / /   \ \   | ||| |
| ||| ||-------------|o|||| |
| ||| ||----\ | /----|o|||| |
| _|| ||-----|||-----|o|||_ |
\/|\/  |     |||     |o|\/|\/
\_o/   |----|||||----|-' \o_/
       |##  |   |  ##|
       |----|   |----|
       ||__ |   | __||
      [|'  `|] [|'  `|]
      [|`--'|] [|`--'|]
      /|__| |\ /| |__|\
      ||  | || || |  ||
      ||__|_|| ||_|__||
      ||    || ||    ||
      \|----|/ \|----|/    -- Puddles Mk VII 
      /______\ /______\
      |__||__| |__||__|
"""
def main():
    rospy.init_node('gate_task')
    sm = smach.StateMachine(outcomes=['done', 'fail'])
    with sm:
        sm.userdata.search_object = 'Gate'
        smach.StateMachine.add('SEARCH', Search(),
                                transitions={'Success': 'done', 'Failure': 'fail'})
        sm.userdata.type = 'gateManuever'
        sm.userdata.args = {}
        smach.StateMachine.add('MOVE', Move())
        sm.execute()

if __name__ == '__main__':
    main()
"""
When Arko tries to get his computer working

                                         .
                                          `.

                                     ...
                                        `.
                                  ..
                                    `.
                            `.        `.
                         ___`.\.//
                            `---.---
                           /     \.--
                          /       \-
                         |   /\    \
                         |\==/\==/  |
                         | `@'`@'  .--.
                  .--------.           )
                .'             .   `._/
               /               |     \
              .               /       |
              |              /        |
              |            .'         |   .--.
             .'`.        .'_          |  /    \
           .'    `.__.--'.--`.       / .'      |
         .'            .|    \\     |_/        |
       .'            .' |     \\               |
     .-`.           /   |      .      __       |
   .'    `.     \   |   `           .'  )      \
  /        \   / \  |            .-'   /       |
 (  /       \ /   \ |                 |        |
  \/         (     \/                 |        |
  (  /        )    /                 /   _.----|
   \/   //   /   .'                  |.-'       `
   (   /(   /   /                    /      `.   |
    `.(  `-')  .---.                |    `.   `._/
       `._.'  /     `.   .---.      |  .   `._.'
              |       \ /     `.     \  `.___.'
              |        Y        `.    `.___.'
              |      . |          \         \
              |       `|           \         |
              |        |       .    \        |
              |        |        \    \       |
            .--.       |         \           |
           /    `.  .----.        \          /
          /       \/      \        \        /
          |       |        \       |       /
           \      |    @    \   `-. \     /
            \      \         \     \|.__.'
             \      \         \     |
              \      \         \    |
               \      \         \   |
                \    .'`.        \  |
                 `.-'    `.    _.'\ |
                   |       `.-'    ||
              .     \     . `.     ||      .'
               `.    `-.-'    `.__.'     .'
                 `.                    .'
             .                       .'
              `.
                                           .-'
                                        .-'

      \                 \
       \         ..      \
        \       /  `-.--.___ __.-.___
`-.      \     /  #   `-._.-'    \   `--.__
   `-.        /  ####    /   ###  \        `.
________     /  #### ############  |       _|           .'
            |\ #### ##############  \__.--' |    /    .'
            | ####################  |       |   /   .'
            | #### ###############  |       |  /
            | #### ###FLEXBE######  |      /|      ----
          . | #### ###############  |    .'<    ____
        .'  | ####################  | _.'-'\|
      .'    |   ##################  |       |
             `.   ################  |       |
               `.    ############   |       | ----
              ___`.     #####     _..____.-'     .
             |`-._ `-._       _.-'    \\\         `.
          .'`-._  `-._ `-._.-'`--.___.-' \          `.
        .' .. . `-._  `-._        ___.---'|   \   \
      .' .. . .. .  `-._  `-.__.-'        |    \   \
     |`-. . ..  . .. .  `-._|             |     \   \
     |   `-._ . ..  . ..   .'            _|
      `-._   `-._ . ..   .' |      __.--'
          `-._   `-._  .' .'|__.--'
              `-._   `' .'
                  `-._.'

                             ____
                     __,-~~/~    `---.
                   _/_,---(      ,    )
               __ /        <    /   )  \___
- ------===;;;'====------------------===;;;===----- -  -
                  \/  ~"~"~"~"~"~\~"~)~"/
                  (_ (   \  (     >    \)
                   \_( _ <         >_>'
                      ~ `-i' ::>|--"
                          I;|.|.|
                         <|i::|i|`.
                        (` ^'"`-' ")

"""
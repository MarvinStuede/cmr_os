#!/usr/bin/env python
"""Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree."""

# -*- coding: utf-8 -*-

from cmr_msgs.srv import SendMessage, SendMessageRequest, SendMessageResponse
from cmr_msgs.msg import *
import rospy
import smtplib
import yaml
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

from cryptography.fernet import Fernet
import os
import sys
import rospkg
from telegram import (ReplyKeyboardMarkup, Bot, KeyboardButton)
from telegram.ext import (Updater, CommandHandler, MessageHandler, Filters,
                          ConversationHandler)
import actionlib
from threading import Lock

#States
FIXED = 1


class MessageNode:
    """Class to send messages via mail or instant messaging. Also provides an action server to send messages via telegram
    and receive answers from supervisors"""

    def __init__(self):
        self.action_mutex = Lock()
        self.received_and_successful = (False, False)
        self.srv = rospy.Service('send_message', SendMessage, self.srv_callback)
        self.msg_to_send = ""
        self.message_cfg = self.load_yaml()
        token=self.message_cfg['telegram']['token']
        self.telegram_bot = Bot(token=token)

        # Get the dispatcher to register handlers
        self.updater = Updater(token, use_context=True)
        self.dp = self.updater.dispatcher
        # Add callbacks to commands, which can be sent by a telegram user
        self.dp.add_handler(CommandHandler('fixed', self.fixed))
        self.dp.add_handler(CommandHandler('not_fixed', self.not_fixed))

        # Start the Bot
        self.updater.start_polling()

        self.acserver = actionlib.SimpleActionServer("call_supervisor", cmr_msgs.msg.CallSupervisorAction, execute_cb=self.supervisor_action_cb, auto_start = False)
        self.acserver.start()

    def supervisor_action_cb(self, goal):
        """Callback for the action server"""
        self.enquiry_telegram(msg_to_send=goal.message)
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            # Use a mutex, dict is also written by telegram callbacks
            self.action_mutex.acquire()
            if self.received_and_successful[0]:
                self.acserver.set_succeeded(cmr_msgs.msg.CallSupervisorResult(self.received_and_successful[1]))
                self.received_and_successful = (False, False)
                self.action_mutex.release()
                break
            self.action_mutex.release()
            r.sleep()

    def fixed(self, update, context):
        """Callback for '/fixed' command. Sets action server result entry 'success' to true"""
        if self.acserver.is_active():
            rospy.loginfo("Supervisor problem fixed!")
            self.action_mutex.acquire()
            self.received_and_successful = (True, True)
            self.action_mutex.release()
            self.send_telegram("Problem fixed!")

        return ConversationHandler.END

    def not_fixed(self, update, context):
        """Callback for '/not_fixed' command. Sets action server result entry 'success' to false"""
        if self.acserver.is_active():
            rospy.loginfo("Supervisor problem not fixed!")
            self.action_mutex.acquire()
            self.received_and_successful = (True, False)
            self.action_mutex.release()

        return ConversationHandler.END

    def send_email(self, msg_to_send):
        """
        Send an email to the user list defined in yaml config file

        Args:
            msg_to_send (String): Message that will be sent

        Returns: True if sent successfully, else False

        """

        try:
            # read in the parameters from yaml

            email_distribution = self.message_cfg['email']['distributor']
            mail_transfer_prot = self.message_cfg['email']['transfer_protocol']
            port = self.message_cfg['email']['port']
            sobi_mail_address = self.message_cfg['email']['robo_mail']
            #sobi_mail_address = "sobi@imes.uni-hannover.de"
            #mail_transfer_prot = "email.uni-hannover.de"
            mail_auth_crypt = self.message_cfg['email']['auth']


            # decrypt the password
            f = Fernet(mail_auth_crypt)
            mail_pw_crypted = os.environ.get('sobi_email_pass')
            mail_pw = f.decrypt(mail_pw_crypted)


            # create the message
            msg = self.create_mail_body(msg_to_send)
            message = msg.as_string()

            # send email to all distributors
            for i in range(len(email_distribution)):
                s = smtplib.SMTP(mail_transfer_prot, port)
                s.starttls()
                s.login(sobi_mail_address, mail_pw)
                s.sendmail(sobi_mail_address, email_distribution[i], message)
                s.quit()
                rospy.loginfo("Email successfully sent to %s", email_distribution[i])
            return True


        except Exception as e:
            rospy.logerr("Something went wrong while sending the message: " + e.message)
            return False

    def enquiry_telegram(self, msg_to_send):
        """
        Send an enquiry via telegram. Gives the options /fixed and /not_fixed which will be shown on the users keyboard.
        Will
        Args:
            msg_to_send ():

        Returns:

        """
        chat_ids = self.message_cfg['telegram']['chat_ids']
        for chat_id in chat_ids:

            # Commands menu
            main_menu_keyboard = [[KeyboardButton('/fixed')],
                                  [KeyboardButton('/not_fixed')]]
            reply_kb_markup = ReplyKeyboardMarkup(main_menu_keyboard,
                                                           resize_keyboard=True,
                                                           one_time_keyboard=True)

            # Send the message with menu
            self.telegram_bot.send_message(chat_id=chat_id,
                             text=msg_to_send,
                             reply_markup=reply_kb_markup)
        return True

    def send_telegram(self, msg_to_send):
        """
        Send the message via telegram
        Args:
            self ():
            msg_to_send (String):

        Returns:
            True
        """
        chat_ids = self.message_cfg['telegram']['chat_ids']
        for chat_id in chat_ids:
            self.telegram_bot.sendMessage(chat_id=chat_id, text=msg_to_send)
            rospy.loginfo("Telegram message successfully sent to %s", chat_id)

        return True

    def srv_callback(self, req):
        """
        Service callback.
        Based on the type defined in request, chooses a service with which the message will be sent
        Args:
            self ():
            req (SendMessageRequest):

        Returns: (SendMessageResponse)

        """
        req.message
        if req.service == SendMessageRequest.EMAIL:
            success = self.send_email(req.message)
        elif req.service == SendMessageRequest.TELEGRAM:
            success = self.send_telegram(req.message)
        else:
            success = False
        return SendMessageResponse(success)


    def load_yaml(self):
        """
        Load the yaml file with config values
        Args:
            self ():

        Returns: Config file

        """
        rospack = rospkg.RosPack()
        rospack.list()
        cfg = yaml.YAMLObject()
        path = rospack.get_path('cmr_api')
        with open(path + "/cfg/message_config.yaml", 'r') as stream:
            try:
                cfg = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        return cfg


    def create_mail_body(self, msg_to_send):
        """
        Insert the message into a HTML Body
        Args:
            self ():
            msg_to_send (String):

        Returns: (String) The mail body

        """
        msg = MIMEMultipart()
        # storing the subject
        msg['Subject'] = "Sobi has something to say"

        # message defined in html code
        body_1 = html = """\
        <html>
            <head></head>
            <body>
                <p> Hello!</p> <br>
                    Message: <br>
                </p>
        """
        body_2 = html = """\

                <p> Regards <br>
                    Sobi
                </p>
            </body>
        </html>
        """
        body = body_1 + msg_to_send + body_2;
        # attach the body with the msg instance
        msg.attach(MIMEText(body, 'html'))
        return msg


def main(args):
    rospy.init_node('send_message', anonymous=False)
    msg_node = MessageNode()
    rospy.loginfo("Send message node started")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Message Node"


if __name__ == '__main__':
    main(sys.argv)

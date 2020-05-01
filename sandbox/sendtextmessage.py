# Script using twilio

from twilio.rest import Client

account_sid = ""
auth_token = ""

client = Client(account_sid, auth_token)
message = client.api.account.messages.create(
			to = "ph number",
			from = "pn number", 
			body = "This is a test message!")
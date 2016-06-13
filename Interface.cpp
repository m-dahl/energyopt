#include "IO.h"
#include "Model.h"
#include "PostProcess.h"

#include <activemq/library/ActiveMQCPP.h>
#include <decaf/lang/Thread.h>
#include <decaf/lang/Runnable.h>
#include <decaf/lang/Integer.h>
#include <decaf/lang/Long.h>
#include <decaf/lang/System.h>
#include <activemq/core/ActiveMQConnectionFactory.h>
#include <activemq/util/Config.h>
#include <cms/Connection.h>
#include <cms/Session.h>
#include <cms/TextMessage.h>
#include <cms/BytesMessage.h>
#include <cms/MapMessage.h>
#include <cms/ExceptionListener.h>
#include <cms/MessageListener.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <memory>
#include <math.h>  

using namespace activemq::core;
using namespace decaf::util::concurrent;
using namespace decaf::util;
using namespace decaf::lang;
using namespace cms;
using namespace std;
using std::pow;

void initializeModel(Model& model, vector<RobotData>& R,
                     PostProcess& postProcess, DataFactory& dataFactory);

// ModalaAMQ.cpp : Defines the entry point for the console application.
//void modalar(string)
class producer {
private:

	Connection* connection;
	Session* session;
	Destination* destination;
	MessageProducer* mproducer;
	int numMessages;
	bool sessionTransacted;
	std::string brokerURI;

private:

	producer(const producer&);
	producer& operator=(const producer&);

public:

	producer(const std::string& brokerURI, bool sessionTransacted = false) :
		connection(NULL),
		session(NULL),
		destination(NULL),
		mproducer(NULL),
		sessionTransacted(sessionTransacted),
		brokerURI(brokerURI) {
	}

	virtual ~producer() {
		cleanup();
	}

	void close() {
		this->cleanup();
	}

	void setup() {
		try {

			// Create a ConnectionFactory
      std::auto_ptr<ConnectionFactory> connectionFactory(
				ConnectionFactory::createCMSConnectionFactory(brokerURI));

			// Create a Connection
			connection = connectionFactory->createConnection();
			connection->start();

			// Create a Session
			if (this->sessionTransacted) {
				session = connection->createSession(Session::SESSION_TRANSACTED);
			}
			else {
				session = connection->createSession(Session::AUTO_ACKNOWLEDGE);
			}

			// Create the destination (Topic or Queue)
			destination = session->createTopic("MODALA.RESPONSES");

			// Create a MessageProducer from the Session to the Topic or Queue
			mproducer = session->createProducer(destination);
			mproducer->setDeliveryMode(DeliveryMode::NON_PERSISTENT);
		}
		catch (CMSException& e) {
			e.printStackTrace();
		}
	}

	void send(std::string text) {
		try {
			std::auto_ptr<TextMessage> message(session->createTextMessage(text));
			mproducer->send(message.get());
		}
		catch (CMSException& e) {
			e.printStackTrace();
		}
	}

private:

	void cleanup() {

		if (connection != NULL) {
			try {
				connection->close();
			}
			catch (cms::CMSException& ex) {
				ex.printStackTrace();
			}
		}

		// Destroy resources.
		try {
			if(destination) delete destination;
			destination = NULL;
			if(mproducer) delete mproducer;
			mproducer = NULL;
			if(session) delete session;
			session = NULL;
			if(connection) delete connection;
			connection = NULL;
		}
		catch (CMSException& e) {
			e.printStackTrace();
		}
	}
};

class consumer : public ExceptionListener,
	public MessageListener,
	public Runnable {

private:

	Connection* connection;
	Session* session;
	Destination* destination;
	MessageConsumer* mconsumer;
	producer *prod;
	bool useTopic;
	bool sessionTransacted;
	std::string brokerURI;

private:

	consumer(const consumer&);
	consumer& operator=(const consumer&);

public:

	consumer(const std::string& brokerURI, producer* p, bool sessionTransacted = false) :
		connection(NULL),
		session(NULL),
		destination(NULL),
		mconsumer(NULL),
		prod(p),
		sessionTransacted(sessionTransacted),
		brokerURI(brokerURI) {
	} 

	virtual ~consumer() {
		cleanup();
	}

	void close() {
		this->cleanup();
	}

	virtual void run() {

		try {

			// Create a ConnectionFactory
			auto_ptr<ConnectionFactory> connectionFactory(
				ConnectionFactory::createCMSConnectionFactory(brokerURI));

			// Create a Connection
			connection = connectionFactory->createConnection();
			connection->start();
			connection->setExceptionListener(this);

			// Create a Session
			if (this->sessionTransacted == true) {
				session = connection->createSession(Session::SESSION_TRANSACTED);
			}
			else {
				session = connection->createSession(Session::AUTO_ACKNOWLEDGE);
			}

			destination = session->createTopic("MODALA.QUERIES");

			// Create a MessageConsumer from the Session to the Topic or Queue
			mconsumer = session->createConsumer(destination);

			mconsumer->setMessageListener(this);

			std::cout.flush();
			std::cerr.flush();
			printf("Connected, listening...\n");
			while (true) {
				// just listen for messages
        Thread::sleep(100);
			}

		}
		catch (CMSException& e) {
			e.printStackTrace();
		}
	}

	// Called from the consumer since this class is a registered MessageListener.
	virtual void onMessage(const Message* message) {		
		printf("Got a message...\n");
		try {
			const TextMessage* textMessage = dynamic_cast<const TextMessage*> (message);

			if (textMessage != NULL) {
				std::string text = textMessage->getText();

				// THIS IS WHERE THE OPTIMIZER SHOULD BE CALLED

				// 1. Parse string as json. Check request type and input data
				// 2. Start optimizer (give it the producer so that it can output progress messages)
				// 3. Have optimizer send progress messages every n:th iteration, or second, or something
				// 4. Create json containing final result. Have the producer send the result again.

				// this should be the input

				DataFactory dataFactory(text);

				Model model;
				PostProcess postProcess;
        vector<RobotData> R;
				prod->send("progress");
        
				initializeModel(model, R, postProcess, dataFactory);

				postProcess.writeAsci();

				ResultFactory result(postProcess);
				result.makeString();

				prod->send(result.getJsonString());
			}
		}
		catch (Exception& e) {
			e.printStackTrace();
		}

		// Commit all messages.
		if (this->sessionTransacted) {
			session->commit();
		}
	}

	// If something bad happens you see it here as this class is also been
	// registered as an ExceptionListener with the connection.
	virtual void onException(const CMSException& ex AMQCPP_UNUSED) {
		printf("CMS Exception occurred.  Shutting down client.\n");
		ex.printStackTrace();
		exit(1);
	}

private:

	void cleanup() {
		if (connection != NULL) {
			try {
				connection->close();
			}
			catch (cms::CMSException& ex) {
				ex.printStackTrace();
			}
		}

		// Destroy resources.
		try {
			if(destination) delete destination;
			destination = NULL;
			if(mconsumer) delete mconsumer;
			mconsumer = NULL;
			if(session) delete session;
			session = NULL;
			if(connection) delete connection;
			connection = NULL;
		}
		catch (CMSException& e) {
			e.printStackTrace();
		}
	}
};


int main(int argc, char** argv)
{
  if(argc < 2) {
    printf("Usage: energyopt busaddress [port=61616]\n");
    return 0;
  }
  std::string brokerURI = "nio://" + std::string(argv[1]);
  if(argc == 3) {
    brokerURI += std::string(":") + std::string(argv[2]);
  } else {
    brokerURI += std::string(":61616");
  }

	// setup
	try
	{
		activemq::library::ActiveMQCPP::initializeLibrary();
		
		producer producer(brokerURI);
		producer.setup();
		consumer consumer(brokerURI, &producer);

		Thread consumerThread(&consumer);
		consumerThread.start();

		consumerThread.join();  // will never reach here

								// cleanup
		consumer.close();
		producer.close();

	}
	catch (...)
	{
		cout << "An Unknown error occured.";
	}
	
  activemq::library::ActiveMQCPP::shutdownLibrary();	

	return 0;
}




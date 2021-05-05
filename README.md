**Overview** 

(a) This ros package contains a service for speech-to-text (can also be extended for text-to-speech with a very little effort). 

(b) It contains two main files: server and client. 

    1. The voice_recognition_service is called by voice_recognition_client. 
    
    2. The interaction happens via speech_to_text service. 
    
    3. The service is expected to give a response as what it hears from the user, as responce status. 
    
    4. Note that we can remove the input text part from the service, currently it is not in use.
  
(c) The service gives you a responce with voice and asks you to give your voice command, "Tell me, What do you want?"

(d) Give your voice command after that, wait for what it hears, which will also be the final responce of the service

**Package Dependencies/Required Installation Required**

1. pip install ffmpy

2. sudo apt-get install python-pyaudio

3. pip install requests

4. pip install monotonic

5. sudo apt-get install ffmpeg

**Running**

Usual: run the client and server files written in python.

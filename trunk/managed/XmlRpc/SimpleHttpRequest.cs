using System;
using System.IO;
using System.Net.Sockets;
using System.Collections;

namespace Nwc.XmlRpc
{
    ///<summary>Very basic HTTP request handler.</summary>
    ///<remarks>This class is designed to accept a TcpClient and treat it as an HTTP request.
    /// It will do some basic header parsing and manage the input and output streams associated
    /// with the request.</remarks>
    public class SimpleHttpRequest
    {
        private String _httpMethod = null;
        private String _protocol;
        private String _filePathFile = null;
        private String _filePathDir = null;
        private String __filePath;
        private readonly TcpClient _client;
        private readonly StreamReader _input;
        private readonly StreamWriter _output;
        private Hashtable _headers;

        /// <summary>A constructor which accepts the TcpClient.</summary>
        /// <remarks>It creates the associated input and output streams, determines the request type,
        /// and parses the remaining HTTP header.</remarks>
        /// <param name="client">The <c>TcpClient</c> associated with the HTTP connection.</param>
        public SimpleHttpRequest(TcpClient client)
        {
            _client = client;
            _output = new StreamWriter(client.GetStream());
            _input = new StreamReader(client.GetStream());
            GetRequestMethod();
            GetRequestHeaders();
        }

        /// <summary>The output <c>StreamWriter</c> associated with the request.</summary>
        public StreamWriter Output
        {
            get
            {
                return _output;
            }
        }

        /// <summary>The input <c>StreamReader</c> associated with the request.</summary>
        public StreamReader Input
        {
            get
            {
                return _input;
            }
        }

        /// <summary>The <c>TcpClient</c> with the request.</summary>
        public TcpClient Client
        {
            get
            {
                return _client;
            }
        }

        private String _filePath
        {
            get
            {
                return __filePath;
            }
            set
            {
                __filePath = value;
                _filePathDir = null;
                _filePathFile = null;
            }
        }

        /// <summary>The type of HTTP request (i.e. PUT, GET, etc.).</summary>
        public String HttpMethod
        {
            get
            {
                return _httpMethod;
            }
        }

        /// <summary>The level of the HTTP protocol.</summary>
        public String Protocol
        {
            get
            {
                return _protocol;
            }
        }

        /// <summary>The "path" which is part of any HTTP request.</summary>
        public String FilePath
        {
            get
            {
                return _filePath;
            }
        }

        /// <summary>The file portion of the "path" which is part of any HTTP request.</summary>
        public String FilePathFile
        {
            get
            {
                if(_filePathFile is not null)
                    return _filePathFile;

                int i = FilePath.LastIndexOf('/');

                if(i == -1)
                    return "";

                i++;
                _filePathFile = FilePath[i..];
                return _filePathFile;
            }
        }

        /// <summary>The directory portion of the "path" which is part of any HTTP request.</summary>
        public String FilePathDir
        {
            get
            {
                if(_filePathDir is not null)
                    return _filePathDir;

                int i = FilePath.LastIndexOf('/');
                if(i == -1)
                    return "";

                i++;
                _filePathDir = FilePath[..i];
                return _filePathDir;
            }
        }

        private void GetRequestMethod()
        {
            string req = _input.ReadLine();
            if(req is null)
                throw new ApplicationException("Void request.");
            var sreq = req.AsSpan();

            if(sreq.StartsWith("GET "))
            {
                _httpMethod = "GET";
                sreq = sreq[5..];
            }
            else if(sreq.StartsWith("POST "))
            {
                _httpMethod = "POST";
                sreq = sreq[6..];
            }
            else
                throw new InvalidOperationException("Unrecognized method in query: " + req);

            sreq = sreq.TrimEnd();
            if(sreq.Length == 0)
                throw new ApplicationException("What do you want?");

            int idx2 = sreq.IndexOf(' ');
            if(idx2 == -1)
            {
                _filePath = sreq.ToString();
                _protocol = string.Empty;
            }
            else
            {
                _filePath = sreq[..idx2].Trim().ToString();
                _protocol = sreq[idx2..].Trim().ToString();
            }
        }

        private void GetRequestHeaders()
        {
            String line;
            int idx;

            _headers = new Hashtable();

            while((line = _input.ReadLine()) != "")
            {
                if(line == null)
                {
                    break;
                }

                idx = line.IndexOf(':');
                if(idx == -1 || idx == line.Length - 1)
                {
                    Logger.WriteEntry("Malformed header line: " + line,LogLevel.Information);
                    continue;
                }

                String key = line[..idx];
                String value = line[(idx + 1)..];

                try
                {
                    _headers.Add(key,value);
                }
                catch(Exception)
                {
                    Logger.WriteEntry("Duplicate header key in line: " + line,LogLevel.Information);
                }
            }
        }

        /// <summary>
        /// Format the object contents into a useful string representation.
        /// </summary>
        ///<returns><c>String</c> representation of the <c>SimpleHttpRequest</c> as the <i>HttpMethod FilePath Protocol</i>.</returns>
        override public String ToString()
        {
            return HttpMethod + " " + FilePath + " " + Protocol;
        }

        /// <summary>
        /// Close the <c>SimpleHttpRequest</c>. This flushes and closes all associated io streams.
        /// </summary>
        public void Close()
        {
            _output.Flush();
            _output.Close();
            _input.Close();
            _client.Close();
        }
    }
}

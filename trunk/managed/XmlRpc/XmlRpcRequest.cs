using System;
using System.Collections;
using System.IO;
using System.Xml;
using System.Net;
using System.Net.Security;
using System.Text;
using System.Net.Http;
using System.Threading;
using System.Linq;
using System.Threading.Tasks;

namespace Nwc.XmlRpc
{
    /// <summary>Class supporting the request side of an XML-RPC transaction.</summary>
    public class XmlRpcRequest
    {
        private string m_methodName = null;
        private readonly Encoding m_encoding = new UTF8Encoding();
        private readonly XmlRpcRequestSerializer _serializer = new();
        private readonly XmlRpcResponseDeserializer _deserializer = new();

        /// <summary><c>ArrayList</c> containing the parameters.</summary>
        protected IList _params = null;

        /// <summary>Instantiate an <c>XmlRpcRequest</c></summary>
        public XmlRpcRequest()
        {
            _params = new ArrayList();
        }

        public XmlRpcRequest(Encoding enc)
        {
            _params = new ArrayList();
            m_encoding = enc;
        }
        /// <summary>Instantiate an <c>XmlRpcRequest</c> for a specified method and parameters.</summary>
        /// <param name="methodName"><c>String</c> designating the <i>object.method</i> on the server the request
        /// should be directed to.</param>
        /// <param name="parameters"><c>ArrayList</c> of XML-RPC type parameters to invoke the request with.</param>
        public XmlRpcRequest(string methodName, IList parameters)
        {
            MethodName = methodName;
            _params = parameters;
        }

        /// <summary><c>ArrayList</c> conntaining the parameters for the request.</summary>
        public virtual IList Params
        {
            get
            {
                return _params;
            }
        }

        /// <summary><c>String</c> conntaining the method name, both object and method, that the request will be sent to.</summary>
        public virtual string MethodName
        {
            get
            {
                return m_methodName;
            }
            set
            {
                m_methodName = value;
            }
        }

        /// <summary><c>String</c> object name portion of the method name.</summary>
        public string MethodNameObject
        {
            get
            {
                int index = MethodName.IndexOf('.');

                if (index == -1)
                    return MethodName;

                return MethodName[..index];
            }
        }

        /// <summary><c>String</c> method name portion of the object.method name.</summary>
        public string MethodNameMethod
        {
            get
            {
                int index = MethodName.IndexOf('.');

                if (index == -1)
                    return MethodName;

                return MethodName.Substring(index + 1);
            }
        }

        /// <summary>Invoke this request on the server.</summary>
        /// <param name="url"><c>String</c> The url of the XML-RPC server.</param>
        /// <returns><c>Object</c> The value returned from the method invocation on the server.</returns>
        /// <exception cref="XmlRpcException">If an exception generated on the server side.</exception>
        public object Invoke(string url, HttpClient client)
        {
            XmlRpcResponse res = Send(url, client);

            if (res.IsFault)
                throw new XmlRpcException(res.FaultCode, res.FaultString);

            return res.Value;
        }

        public object Invoke(string url, RemoteCertificateValidationCallback certCallBack = null)
        {
            XmlRpcResponse res = Send(url, 100000, certCallBack);

            if (res.IsFault)
                throw new XmlRpcException(res.FaultCode, res.FaultString);

            return res.Value;
        }

        /// <summary>Send the request to the server.</summary>
        /// <param name="url"><c>String</c> The url of the XML-RPC server.</param>
        /// <returns><c>XmlRpcResponse</c> The response generated.</returns>
        public XmlRpcResponse Send(string url, int timeout = 100000, RemoteCertificateValidationCallback certCallBack = null)
        {
            HttpWebRequest request = (HttpWebRequest)WebRequest.Create(url);
            if (request == null)
                throw new XmlRpcException(XmlRpcErrorCodes.TRANSPORT_ERROR,
                              XmlRpcErrorCodes.TRANSPORT_ERROR_MSG + ": Could not create request with " + url);
            request.Method = "POST";
            request.ContentType = "text/xml";
            request.AllowWriteStreamBuffering = true;
            request.Timeout = timeout;
            if(certCallBack != null)
                request.ServerCertificateValidationCallback = certCallBack;

            using (Stream stream = request.GetRequestStream())
            using (XmlTextWriter xml = new(stream, m_encoding))
            {
                _serializer.Serialize(xml, this);
                xml.Flush();
            }

            XmlRpcResponse resp;
            using (HttpWebResponse response = (HttpWebResponse)request.GetResponse())
            using (StreamReader input = new(response.GetResponseStream()))
                resp = (XmlRpcResponse)_deserializer.Deserialize(input);
            return resp;
        }

        public XmlRpcResponse Send(string url, HttpClient client)
        {
            HttpResponseMessage responseMessage = null;
            HttpRequestMessage request = null;
            try
            {
                request = new(HttpMethod.Post, url);
                request.Headers.ExpectContinue = false;
                request.Headers.TransferEncodingChunked = false;

                //if (keepalive)
                {
                    request.Headers.TryAddWithoutValidation("Keep-Alive", "timeout=30, max=10");
                    request.Headers.TryAddWithoutValidation("Connection", "Keep-Alive");
                }
                //else
                //    request.Headers.TryAddWithoutValidation("Connection", "close");

                byte[] outbuf;
                using (MemoryStream outbufms = new())
                using (XmlTextWriter xml = new(outbufms, m_encoding))
                {
                    _serializer.Serialize(xml, this);
                    xml.Flush();
                    outbuf = outbufms.ToArray();
                }

                request.Content = new ByteArrayContent(outbuf);
                request.Content.Headers.TryAddWithoutValidation("Content-Type", "text/xml");
                request.Content.Headers.TryAddWithoutValidation("Content-Length", outbuf.Length.ToString());

                responseMessage = client.Send(request, HttpCompletionOption.ResponseHeadersRead);
                responseMessage.EnsureSuccessStatusCode();

                XmlRpcResponse resp;

                using (StreamReader input = new(responseMessage.Content.ReadAsStream()))
                    resp = (XmlRpcResponse)_deserializer.Deserialize(input);
                return resp;
            }
            catch (WebException ex) when (ex.Status is WebExceptionStatus.Timeout)
            {
                throw new HttpRequestException("request timeout");
            }
            //catch (TaskCanceledException ex) when (ex.InnerException is TimeoutException)
            catch (TaskCanceledException)
            {
                throw new HttpRequestException("request timeout");
            }
            finally
            {
                request?.Dispose();
                responseMessage?.Dispose();
            }
        }

        /// <summary>Produce <c>String</c> representation of the object.</summary>
        /// <returns><c>String</c> representation of the object.</returns>
        override public string ToString()
        {
            return _serializer.Serialize(this);
        }
    }
}

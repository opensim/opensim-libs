
using System;
using System.Collections;
using System.IO;
using System.Xml;
using System.Globalization;

namespace Nwc.XmlRpc
{

    /// <summary>Parser context, we maintain contexts in a stack to avoiding recursion. </summary>
    struct Context
    {
        public string Name;
        public object Container;
    }

    /// <summary>Basic XML-RPC data deserializer.</summary>
    /// <remarks>Uses <c>XmlTextReader</c> to parse the XML data. This level of the class 
    /// only handles the tokens common to both Requests and Responses. This class is not useful in and of itself
    /// but is designed to be subclassed.</remarks>
    public class XmlRpcDeserializer:XmlRpcXmlTokens
    {
        private readonly static DateTimeFormatInfo _dateFormat = new();

        private object _container;
        private Stack _containerStack;

        /// <summary>Protected reference to last text.</summary>
        protected string _text;
        /// <summary>Protected reference to last deserialized value.</summary>
        protected object _value;
        /// <summary>Protected reference to last name field.</summary>
        protected string _name;


        /// <summary>Basic constructor.</summary>
        public XmlRpcDeserializer()
        {
            Reset();
            _dateFormat.FullDateTimePattern = ISO_DATETIME;
        }

        /// <summary>Static method that parses XML data into a response using the Singleton.</summary>
        /// <param name="xmlData"><c>StreamReader</c> containing an XML-RPC response.</param>
        /// <returns><c>Object</c> object resulting from the deserialization.</returns>
        virtual public object Deserialize(TextReader xmlData)
        {
            return null;
        }

        /// <summary>Protected method to parse a node in an XML-RPC XML stream.</summary>
        /// <remarks>Method deals with elements common to all XML-RPC data, subclasses of 
        /// this object deal with request/response spefic elements.</remarks>
        /// <param name="reader"><c>XmlTextReader</c> of the in progress parsing data stream.</param>
        protected void DeserializeNode(XmlTextReader reader)
        {
            switch(reader.NodeType)
            {
                case XmlNodeType.Element:
                    if(Logger.Delegate is not null)
                        Logger.WriteEntry("START " + reader.Name,LogLevel.Information);
                    switch(reader.Name)
                    {
                        case VALUE:
                            _value = null;
                            _text = null;
                            break;
                        case STRUCT:
                            PushContext();
                            _container = new Hashtable();
                            break;
                        case ARRAY:
                            PushContext();
                            _container = new ArrayList();
                            break;
                    }
                    break;
                case XmlNodeType.EndElement:
                    if(Logger.Delegate is not null)
                        Logger.WriteEntry("END " + reader.Name,LogLevel.Information);
                    switch(reader.Name)
                    {
                        case BASE64:
                            if(string.IsNullOrEmpty(_text))
                                _value = Array.Empty<byte>();
                            else
                                _value = Convert.FromBase64String(_text);
                            break;
                        case BOOLEAN:
                            int val = Int16.Parse(_text);
                            if(val == 0)
                                _value = false;
                            else if(val == 1)
                                _value = true;
                            break;
                        case STRING:
                            _value = _text;
                            break;
                        case DOUBLE:
                            _value = Double.Parse(_text,CultureInfo.InvariantCulture);
                            break;
                        case INT:
                        case ALT_INT:
                            _value = Int32.Parse(_text);
                            break;
                        case DATETIME:
                            _value = DateTime.ParseExact(_text,"F",_dateFormat);
                            break;
                        case NAME:
                            _name = _text;
                            break;
                        case VALUE:
                            _value ??= _text; // some kits don't use <string> tag, they just do <value>
                            if(_container is IList cnt) // in an array?  If so add value to it.
                                cnt.Add(_value);
                            break;
                        case MEMBER:
                            if(_container is IDictionary icnt) // in an struct?  If so add value to it.
                                icnt.Add(_name,_value);
                            break;
                        case ARRAY:
                        case STRUCT:
                            _value = _container;
                            PopContext();
                            break;
                    }
                    break;
                case XmlNodeType.Text:
                    if(Logger.Delegate is not null)
                        Logger.WriteEntry("Text " + reader.Value,LogLevel.Information);
                    _text = reader.Value;
                    break;
                default:
                    break;
            }
        }

        /// <summary>Static method that parses XML in a <c>String</c> into a 
        /// request using the Singleton.</summary>
        /// <param name="xmlData"><c>String</c> containing an XML-RPC request.</param>
        /// <returns><c>XmlRpcRequest</c> object resulting from the parse.</returns>
        public object Deserialize(string xmlData)
        {
            return Deserialize(new StringReader(xmlData));
        }

        /// <summary>Pop a Context of the stack, an Array or Struct has closed.</summary>
        private void PopContext()
        {
            Context c = (Context)_containerStack.Pop();
            _container = c.Container;
            _name = c.Name;
        }

        /// <summary>Push a Context on the stack, an Array or Struct has opened.</summary>
        private void PushContext()
        {
            Context context;

            context.Container = _container;
            context.Name = _name;

            _containerStack.Push(context);
        }

        /// <summary>Reset the internal state of the deserializer.</summary>
        protected void Reset()
        {
            _text = null;
            _value = null;
            _name = null;
            _container = null;
            _containerStack = new Stack();
        }
    }
}



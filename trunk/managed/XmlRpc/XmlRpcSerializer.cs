
using System;
using System.Text;
using System.Collections;
using System.IO;
using System.Xml;
using System.Globalization;

namespace Nwc.XmlRpc
{
    /// <summary>Base class of classes serializing data to XML-RPC's XML format.</summary>
    /// <remarks>This class handles the basic type conversions like Integer to &lt;i4&gt;. </remarks>
    /// <seealso cref="XmlRpcXmlTokens"/>
    public class XmlRpcSerializer : XmlRpcXmlTokens
    {
        private readonly Encoding m_encoding = new UTF8Encoding();
        /// <summary>Serialize the <c>XmlRpcRequest</c> to the output stream.</summary>
        /// <param name="output">An <c>XmlTextWriter</c> stream to write data to.</param>
        /// <param name="obj">An <c>Object</c> to serialize.</param>
        /// <seealso cref="XmlRpcRequest"/>

        virtual public void Serialize(XmlTextWriter output, Object obj)
        {
        }

        /// <summary>Serialize the <c>XmlRpcRequest</c> to a String.</summary>
        /// <remarks>Note this may represent a real memory hog for a large request.</remarks>
        /// <param name="obj">An <c>Object</c> to serialize.</param>
        /// <returns><c>String</c> containing XML-RPC representation of the request.</returns>
        /// <seealso cref="XmlRpcRequest"/>
        public String Serialize(Object obj)
        {
            using MemoryStream ms = new(1024);
            using XmlTextWriter xml = new(ms, m_encoding);
            xml.Formatting = Formatting.Indented;
            xml.Indentation = 4;
            Serialize(xml, obj);
            xml.Flush();
            return m_encoding.GetString(ms.ToArray());
        }

        /// <remarks>Serialize the object to the output stream.</remarks>
        /// <param name="output">An <c>XmlTextWriter</c> stream to write data to.</param>
        /// <param name="obj">An <c>Object</c> to serialize.</param>
        public void SerializeObject(XmlTextWriter output, Object obj)
        {
            if (obj is null)
                return;

            if (obj is byte[] ba)
            {
                output.WriteStartElement(BASE64);
                output.WriteBase64(ba, 0, ba.Length);
                output.WriteEndElement();
            }
            else if (obj is String)
            {
                output.WriteElementString(STRING, obj.ToString());
            }
            else if (obj is Int32)
            {
                output.WriteElementString(INT, obj.ToString());
            }
            else if (obj is DateTime dt)
            {
                output.WriteElementString(DATETIME, dt.ToString(ISO_DATETIME));
            }
            else if (obj is Double dbl)
            {
                output.WriteElementString(DOUBLE, dbl.ToString(CultureInfo.InvariantCulture));
            }
            else if (obj is Boolean bl)
            {
                output.WriteElementString(BOOLEAN, bl == true ? "1" : "0");
            }
            else if (obj is IList lst)
            {
                output.WriteStartElement(ARRAY);
                output.WriteStartElement(DATA);
                if (((ArrayList)obj).Count > 0)
                {
                    foreach (Object member in lst)
                    {
                        output.WriteStartElement(VALUE);
                        SerializeObject(output, member);
                        output.WriteEndElement();
                    }
                }
                output.WriteEndElement();
                output.WriteEndElement();
            }
            else if (obj is IDictionary h)
            {
                output.WriteStartElement(STRUCT);
                foreach (String key in h.Keys)
                {
                    output.WriteStartElement(MEMBER);
                    output.WriteElementString(NAME, key);
                    output.WriteStartElement(VALUE);
                    SerializeObject(output, h[key]);
                    output.WriteEndElement();
                    output.WriteEndElement();
                }
                output.WriteEndElement();
            }
        }
    }
}

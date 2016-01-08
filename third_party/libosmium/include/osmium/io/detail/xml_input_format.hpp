#ifndef OSMIUM_IO_DETAIL_XML_INPUT_FORMAT_HPP
#define OSMIUM_IO_DETAIL_XML_INPUT_FORMAT_HPP

/*

This file is part of Osmium (http://osmcode.org/libosmium).

Copyright 2013-2015 Jochen Topf <jochen@topf.org> and others (see README).

Boost Software License - Version 1.0 - August 17th, 2003

Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of the software and accompanying documentation covered by
this license (the "Software") to use, reproduce, display, distribute,
execute, and transmit the Software, and to prepare derivative works of the
Software, and to permit third-parties to whom the Software is furnished to
do so, all subject to the following:

The copyright notices in the Software and this entire statement, including
the above license grant, this restriction and the following disclaimer,
must be included in all copies of the Software, in whole or in part, and
all derivative works of the Software, unless such copies or derivative
works are solely in the form of machine-executable object code generated by
a source language processor.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.

*/

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <future>
#include <iostream>
#include <memory>
#include <ratio>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <expat.h>

#include <osmium/builder/builder.hpp>
#include <osmium/builder/osm_object_builder.hpp>
#include <osmium/io/detail/input_format.hpp>
#include <osmium/io/error.hpp>
#include <osmium/io/file_format.hpp>
#include <osmium/io/header.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm.hpp>
#include <osmium/osm/box.hpp>
#include <osmium/osm/entity_bits.hpp>
#include <osmium/osm/item_type.hpp>
#include <osmium/osm/location.hpp>
#include <osmium/osm/object.hpp>
#include <osmium/osm/types.hpp>
#include <osmium/osm/types_from_string.hpp>
#include <osmium/thread/queue.hpp>
#include <osmium/thread/util.hpp>
#include <osmium/util/cast.hpp>

namespace osmium {

    /**
     * Exception thrown when the XML parser failed. The exception contains
     * (if available) information about the place where the error happened
     * and the type of error.
     */
    struct xml_error : public io_error {

        unsigned long line;
        unsigned long column;
        XML_Error error_code;
        std::string error_string;

        explicit xml_error(XML_Parser parser) :
            io_error(std::string("XML parsing error at line ")
                    + std::to_string(XML_GetCurrentLineNumber(parser))
                    + ", column "
                    + std::to_string(XML_GetCurrentColumnNumber(parser))
                    + ": "
                    + XML_ErrorString(XML_GetErrorCode(parser))),
            line(XML_GetCurrentLineNumber(parser)),
            column(XML_GetCurrentColumnNumber(parser)),
            error_code(XML_GetErrorCode(parser)),
            error_string(XML_ErrorString(error_code)) {
        }

        explicit xml_error(const std::string& message) :
            io_error(message),
            line(0),
            column(0),
            error_code(),
            error_string(message) {
        }

    }; // struct xml_error

    /**
     * Exception thrown when an OSM XML files contains no version attribute
     * on the 'osm' element or if the version is unknown.
     */
    struct format_version_error : public io_error {

        std::string version;

        explicit format_version_error() :
            io_error("Can not read file without version (missing version attribute on osm element)."),
            version() {
        }

        explicit format_version_error(const char* v) :
            io_error(std::string("Can not read file with version ") + v),
            version(v) {
        }

    }; // struct format_version_error

    namespace io {

        class File;

        namespace detail {

            /**
             * Once the header is fully parsed this exception will be thrown if
             * the caller is not interested in anything else except the header.
             * It will break off the parsing at this point.
             *
             * This exception is never seen by user code, it is caught internally.
             */
            class ParserIsDone : std::exception {
            };

            class XMLParser {

                static constexpr int buffer_size = 10 * 1000 * 1000;

                enum class context {
                    root,
                    top,
                    node,
                    way,
                    relation,
                    changeset,
                    ignored_node,
                    ignored_way,
                    ignored_relation,
                    ignored_changeset,
                    in_object
                }; // enum class context

                context m_context;
                context m_last_context;

                /**
                 * This is used only for change files which contain create, modify,
                 * and delete sections.
                 */
                bool m_in_delete_section;

                osmium::io::Header m_header;

                osmium::memory::Buffer m_buffer;

                std::unique_ptr<osmium::builder::NodeBuilder>               m_node_builder;
                std::unique_ptr<osmium::builder::WayBuilder>                m_way_builder;
                std::unique_ptr<osmium::builder::RelationBuilder>           m_relation_builder;
                std::unique_ptr<osmium::builder::ChangesetBuilder>          m_changeset_builder;

                std::unique_ptr<osmium::builder::TagListBuilder>            m_tl_builder;
                std::unique_ptr<osmium::builder::WayNodeListBuilder>        m_wnl_builder;
                std::unique_ptr<osmium::builder::RelationMemberListBuilder> m_rml_builder;

                osmium::thread::Queue<std::string>& m_input_queue;
                osmium::thread::Queue<osmium::memory::Buffer>& m_queue;
                std::promise<osmium::io::Header>& m_header_promise;

                osmium::osm_entity_bits::type m_read_types;

                std::atomic<bool>& m_done;

                bool m_header_is_done;

                /**
                 * A C++ wrapper for the Expat parser that makes sure no memory is leaked.
                 */
                template <class T>
                class ExpatXMLParser {

                    XML_Parser m_parser;

                    static void XMLCALL start_element_wrapper(void* data, const XML_Char* element, const XML_Char** attrs) {
                        static_cast<XMLParser*>(data)->start_element(element, attrs);
                    }

                    static void XMLCALL end_element_wrapper(void* data, const XML_Char* element) {
                        static_cast<XMLParser*>(data)->end_element(element);
                    }

                public:

                    ExpatXMLParser(T* callback_object) :
                        m_parser(XML_ParserCreate(nullptr)) {
                        if (!m_parser) {
                            throw osmium::io_error("Internal error: Can not create parser");
                        }
                        XML_SetUserData(m_parser, callback_object);
                        XML_SetElementHandler(m_parser, start_element_wrapper, end_element_wrapper);
                    }

                    ExpatXMLParser(const ExpatXMLParser&) = delete;
                    ExpatXMLParser(ExpatXMLParser&&) = delete;

                    ExpatXMLParser& operator=(const ExpatXMLParser&) = delete;
                    ExpatXMLParser& operator=(ExpatXMLParser&&) = delete;

                    ~ExpatXMLParser() {
                        XML_ParserFree(m_parser);
                    }

                    void operator()(const std::string& data, bool last) {
                        if (XML_Parse(m_parser, data.data(), static_cast_with_assert<int>(data.size()), last) == XML_STATUS_ERROR) {
                            throw osmium::xml_error(m_parser);
                        }
                    }

                }; // class ExpatXMLParser

                /**
                 * A helper class that makes sure a promise is kept. It stores
                 * a reference to some piece of data and to a promise and, on
                 * destruction, sets the value of the promise from the data.
                 */
                template <class T>
                class PromiseKeeper {

                    T& m_data;
                    std::promise<T>& m_promise;
                    bool m_done;

                public:

                    PromiseKeeper(T& data, std::promise<T>& promise) :
                        m_data(data),
                        m_promise(promise),
                        m_done(false) {
                    }

                    void fullfill_promise() {
                        if (!m_done) {
                            m_promise.set_value(m_data);
                            m_done = true;
                        }
                    }

                    ~PromiseKeeper() {
                        fullfill_promise();
                    }

                }; // class PromiseKeeper

            public:

                explicit XMLParser(osmium::thread::Queue<std::string>& input_queue, osmium::thread::Queue<osmium::memory::Buffer>& queue, std::promise<osmium::io::Header>& header_promise, osmium::osm_entity_bits::type read_types, std::atomic<bool>& done) :
                    m_context(context::root),
                    m_last_context(context::root),
                    m_in_delete_section(false),
                    m_header(),
                    m_buffer(buffer_size),
                    m_node_builder(),
                    m_way_builder(),
                    m_relation_builder(),
                    m_changeset_builder(),
                    m_tl_builder(),
                    m_wnl_builder(),
                    m_rml_builder(),
                    m_input_queue(input_queue),
                    m_queue(queue),
                    m_header_promise(header_promise),
                    m_read_types(read_types),
                    m_done(done),
                    m_header_is_done(false) {
                }

                /**
                 * The copy constructor is needed for storing XMLParser in a std::function.
                 * The copy will look the same as if it has been initialized with the
                 * same parameters as the original. Any state changes in the original will
                 * not be reflected in the copy.
                 */
                XMLParser(const XMLParser& other) :
                    m_context(context::root),
                    m_last_context(context::root),
                    m_in_delete_section(false),
                    m_header(),
                    m_buffer(buffer_size),
                    m_node_builder(),
                    m_way_builder(),
                    m_relation_builder(),
                    m_changeset_builder(),
                    m_tl_builder(),
                    m_wnl_builder(),
                    m_rml_builder(),
                    m_input_queue(other.m_input_queue),
                    m_queue(other.m_queue),
                    m_header_promise(other.m_header_promise),
                    m_read_types(other.m_read_types),
                    m_done(other.m_done),
                    m_header_is_done(other.m_header_is_done) {
                }

                XMLParser(XMLParser&&) = default;

                XMLParser& operator=(const XMLParser&) = delete;

                XMLParser& operator=(XMLParser&&) = default;

                ~XMLParser() = default;

                bool operator()() {
                    ExpatXMLParser<XMLParser> parser(this);
                    PromiseKeeper<osmium::io::Header> promise_keeper(m_header, m_header_promise);
                    bool last;
                    do {
                        std::string data;
                        m_input_queue.wait_and_pop(data);
                        last = data.empty();
                        try {
                            parser(data, last);
                            if (m_header_is_done) {
                                promise_keeper.fullfill_promise();
                            }
                        } catch (ParserIsDone&) {
                            return true;
                        } catch (...) {
                            m_queue.push(osmium::memory::Buffer()); // empty buffer to signify eof
                            throw;
                        }
                    } while (!last && !m_done);
                    if (m_buffer.committed() > 0) {
                        m_queue.push(std::move(m_buffer));
                    }
                    m_queue.push(osmium::memory::Buffer()); // empty buffer to signify eof
                    return true;
                }

            private:

                const char* init_object(osmium::OSMObject& object, const XML_Char** attrs) {
                    const char* user = "";

                    if (m_in_delete_section) {
                        object.set_visible(false);
                    }

                    osmium::Location location;
                    for (int count = 0; attrs[count]; count += 2) {
                        if (!strcmp(attrs[count], "lon")) {
                            location.set_lon(std::atof(attrs[count+1])); // XXX doesn't detect garbage after the number
                        } else if (!strcmp(attrs[count], "lat")) {
                            location.set_lat(std::atof(attrs[count+1])); // XXX doesn't detect garbage after the number
                        } else if (!strcmp(attrs[count], "user")) {
                            user = attrs[count+1];
                        } else {
                            object.set_attribute(attrs[count], attrs[count+1]);
                        }
                    }

                    if (location && object.type() == osmium::item_type::node) {
                        static_cast<osmium::Node&>(object).set_location(location);
                    }

                    return user;
                }

                void init_changeset(osmium::builder::ChangesetBuilder* builder, const XML_Char** attrs) {
                    const char* user = "";
                    osmium::Changeset& new_changeset = builder->object();

                    osmium::Location min;
                    osmium::Location max;
                    for (int count = 0; attrs[count]; count += 2) {
                        if (!strcmp(attrs[count], "min_lon")) {
                            min.set_lon(atof(attrs[count+1]));
                        } else if (!strcmp(attrs[count], "min_lat")) {
                            min.set_lat(atof(attrs[count+1]));
                        } else if (!strcmp(attrs[count], "max_lon")) {
                            max.set_lon(atof(attrs[count+1]));
                        } else if (!strcmp(attrs[count], "max_lat")) {
                            max.set_lat(atof(attrs[count+1]));
                        } else if (!strcmp(attrs[count], "user")) {
                            user = attrs[count+1];
                        } else {
                            new_changeset.set_attribute(attrs[count], attrs[count+1]);
                        }
                    }

                    new_changeset.bounds().extend(min);
                    new_changeset.bounds().extend(max);

                    builder->add_user(user);
                }

                void check_tag(osmium::builder::Builder* builder, const XML_Char* element, const XML_Char** attrs) {
                    if (!strcmp(element, "tag")) {
                        m_wnl_builder.reset();
                        m_rml_builder.reset();

                        const char* key = "";
                        const char* value = "";
                        for (int count = 0; attrs[count]; count += 2) {
                            if (attrs[count][0] == 'k' && attrs[count][1] == 0) {
                                key = attrs[count+1];
                            } else if (attrs[count][0] == 'v' && attrs[count][1] == 0) {
                                value = attrs[count+1];
                            }
                        }
                        if (!m_tl_builder) {
                            m_tl_builder = std::unique_ptr<osmium::builder::TagListBuilder>(new osmium::builder::TagListBuilder(m_buffer, builder));
                        }
                        m_tl_builder->add_tag(key, value);
                    }
                }

                void header_is_done() {
                    m_header_is_done = true;
                    if (m_read_types == osmium::osm_entity_bits::nothing) {
                        throw ParserIsDone();
                    }
                }

                void start_element(const XML_Char* element, const XML_Char** attrs) {
                    switch (m_context) {
                        case context::root:
                            if (!strcmp(element, "osm") || !strcmp(element, "osmChange")) {
                                if (!strcmp(element, "osmChange")) {
                                    m_header.set_has_multiple_object_versions(true);
                                }
                                for (int count = 0; attrs[count]; count += 2) {
                                    if (!strcmp(attrs[count], "version")) {
                                        m_header.set("version", attrs[count+1]);
                                        if (strcmp(attrs[count+1], "0.6")) {
                                            throw osmium::format_version_error(attrs[count+1]);
                                        }
                                    } else if (!strcmp(attrs[count], "generator")) {
                                        m_header.set("generator", attrs[count+1]);
                                    }
                                }
                                if (m_header.get("version") == "") {
                                    throw osmium::format_version_error();
                                }
                            } else {
                                throw osmium::xml_error(std::string("Unknown top-level element: ") + element);
                            }
                            m_context = context::top;
                            break;
                        case context::top:
                            assert(!m_tl_builder);
                            if (!strcmp(element, "node")) {
                                header_is_done();
                                if (m_read_types & osmium::osm_entity_bits::node) {
                                    m_node_builder = std::unique_ptr<osmium::builder::NodeBuilder>(new osmium::builder::NodeBuilder(m_buffer));
                                    m_node_builder->add_user(init_object(m_node_builder->object(), attrs));
                                    m_context = context::node;
                                } else {
                                    m_context = context::ignored_node;
                                }
                            } else if (!strcmp(element, "way")) {
                                header_is_done();
                                if (m_read_types & osmium::osm_entity_bits::way) {
                                    m_way_builder = std::unique_ptr<osmium::builder::WayBuilder>(new osmium::builder::WayBuilder(m_buffer));
                                    m_way_builder->add_user(init_object(m_way_builder->object(), attrs));
                                    m_context = context::way;
                                } else {
                                    m_context = context::ignored_way;
                                }
                            } else if (!strcmp(element, "relation")) {
                                header_is_done();
                                if (m_read_types & osmium::osm_entity_bits::relation) {
                                    m_relation_builder = std::unique_ptr<osmium::builder::RelationBuilder>(new osmium::builder::RelationBuilder(m_buffer));
                                    m_relation_builder->add_user(init_object(m_relation_builder->object(), attrs));
                                    m_context = context::relation;
                                } else {
                                    m_context = context::ignored_relation;
                                }
                            } else if (!strcmp(element, "changeset")) {
                                header_is_done();
                                if (m_read_types & osmium::osm_entity_bits::changeset) {
                                    m_changeset_builder = std::unique_ptr<osmium::builder::ChangesetBuilder>(new osmium::builder::ChangesetBuilder(m_buffer));
                                    init_changeset(m_changeset_builder.get(), attrs);
                                    m_context = context::changeset;
                                } else {
                                    m_context = context::ignored_changeset;
                                }
                            } else if (!strcmp(element, "bounds")) {
                                osmium::Location min;
                                osmium::Location max;
                                for (int count = 0; attrs[count]; count += 2) {
                                    if (!strcmp(attrs[count], "minlon")) {
                                        min.set_lon(atof(attrs[count+1]));
                                    } else if (!strcmp(attrs[count], "minlat")) {
                                        min.set_lat(atof(attrs[count+1]));
                                    } else if (!strcmp(attrs[count], "maxlon")) {
                                        max.set_lon(atof(attrs[count+1]));
                                    } else if (!strcmp(attrs[count], "maxlat")) {
                                        max.set_lat(atof(attrs[count+1]));
                                    }
                                }
                                osmium::Box box;
                                box.extend(min).extend(max);
                                m_header.add_box(box);
                            } else if (!strcmp(element, "delete")) {
                                m_in_delete_section = true;
                            }
                            break;
                        case context::node:
                            m_last_context = context::node;
                            m_context = context::in_object;
                            check_tag(m_node_builder.get(), element, attrs);
                            break;
                        case context::way:
                            m_last_context = context::way;
                            m_context = context::in_object;
                            if (!strcmp(element, "nd")) {
                                m_tl_builder.reset();

                                if (!m_wnl_builder) {
                                    m_wnl_builder = std::unique_ptr<osmium::builder::WayNodeListBuilder>(new osmium::builder::WayNodeListBuilder(m_buffer, m_way_builder.get()));
                                }

                                for (int count = 0; attrs[count]; count += 2) {
                                    if (!strcmp(attrs[count], "ref")) {
                                        m_wnl_builder->add_node_ref(osmium::string_to_object_id(attrs[count+1]));
                                    }
                                }
                            } else {
                                check_tag(m_way_builder.get(), element, attrs);
                            }
                            break;
                        case context::relation:
                            m_last_context = context::relation;
                            m_context = context::in_object;
                            if (!strcmp(element, "member")) {
                                m_tl_builder.reset();

                                if (!m_rml_builder) {
                                    m_rml_builder = std::unique_ptr<osmium::builder::RelationMemberListBuilder>(new osmium::builder::RelationMemberListBuilder(m_buffer, m_relation_builder.get()));
                                }

                                char type = 'x';
                                object_id_type ref  = 0;
                                const char* role = "";
                                for (int count = 0; attrs[count]; count += 2) {
                                    if (!strcmp(attrs[count], "type")) {
                                        type = static_cast<char>(attrs[count+1][0]);
                                    } else if (!strcmp(attrs[count], "ref")) {
                                        ref = osmium::string_to_object_id(attrs[count+1]);
                                    } else if (!strcmp(attrs[count], "role")) {
                                        role = static_cast<const char*>(attrs[count+1]);
                                    }
                                }
                                // XXX assert type, ref, role are set
                                m_rml_builder->add_member(char_to_item_type(type), ref, role);
                            } else {
                                check_tag(m_relation_builder.get(), element, attrs);
                            }
                            break;
                        case context::changeset:
                            m_last_context = context::changeset;
                            m_context = context::in_object;
                            check_tag(m_changeset_builder.get(), element, attrs);
                            break;
                        case context::ignored_node:
                            break;
                        case context::ignored_way:
                            break;
                        case context::ignored_relation:
                            break;
                        case context::ignored_changeset:
                            break;
                        case context::in_object:
                            assert(false); // should never be here
                            break;
                    }
                }

                void end_element(const XML_Char* element) {
                    switch (m_context) {
                        case context::root:
                            assert(false); // should never be here
                            break;
                        case context::top:
                            if (!strcmp(element, "osm") || !strcmp(element, "osmChange")) {
                                header_is_done();
                                m_context = context::root;
                            } else if (!strcmp(element, "delete")) {
                                m_in_delete_section = false;
                            }
                            break;
                        case context::node:
                            assert(!strcmp(element, "node"));
                            m_tl_builder.reset();
                            m_node_builder.reset();
                            m_buffer.commit();
                            m_context = context::top;
                            flush_buffer();
                            break;
                        case context::way:
                            assert(!strcmp(element, "way"));
                            m_tl_builder.reset();
                            m_wnl_builder.reset();
                            m_way_builder.reset();
                            m_buffer.commit();
                            m_context = context::top;
                            flush_buffer();
                            break;
                        case context::relation:
                            assert(!strcmp(element, "relation"));
                            m_tl_builder.reset();
                            m_rml_builder.reset();
                            m_relation_builder.reset();
                            m_buffer.commit();
                            m_context = context::top;
                            flush_buffer();
                            break;
                        case context::changeset:
                            assert(!strcmp(element, "changeset"));
                            m_tl_builder.reset();
                            m_changeset_builder.reset();
                            m_buffer.commit();
                            m_context = context::top;
                            flush_buffer();
                            break;
                        case context::in_object:
                            m_context = m_last_context;
                            break;
                        case context::ignored_node:
                            if (!strcmp(element, "node")) {
                                m_context = context::top;
                            }
                            break;
                        case context::ignored_way:
                            if (!strcmp(element, "way")) {
                                m_context = context::top;
                            }
                            break;
                        case context::ignored_relation:
                            if (!strcmp(element, "relation")) {
                                m_context = context::top;
                            }
                            break;
                        case context::ignored_changeset:
                            if (!strcmp(element, "changeset")) {
                                m_context = context::top;
                            }
                            break;
                    }
                }

                void flush_buffer() {
                    if (m_buffer.capacity() - m_buffer.committed() < 1000 * 1000) {
                        m_queue.push(std::move(m_buffer));
                        osmium::memory::Buffer buffer(buffer_size);
                        std::swap(m_buffer, buffer);
                    }
                }

            }; // class XMLParser

            class XMLInputFormat : public osmium::io::detail::InputFormat {

                static constexpr size_t max_queue_size = 100;

                osmium::thread::Queue<osmium::memory::Buffer> m_queue;
                std::atomic<bool> m_done;
                std::promise<osmium::io::Header> m_header_promise;
                std::future<bool> m_parser_future;

            public:

                /**
                 * Instantiate XML Parser
                 *
                 * @param file osmium::io::File instance describing file to be read from.
                 * @param read_which_entities Which types of OSM entities (nodes, ways, relations, changesets) should be parsed?
                 * @param input_queue String queue where data is read from.
                 */
                explicit XMLInputFormat(const osmium::io::File& file, osmium::osm_entity_bits::type read_which_entities, osmium::thread::Queue<std::string>& input_queue) :
                    osmium::io::detail::InputFormat(file, read_which_entities),
                    m_queue(max_queue_size, "xml_parser_results"),
                    m_done(false),
                    m_header_promise(),
                    m_parser_future(std::async(std::launch::async, XMLParser(input_queue, m_queue, m_header_promise, read_which_entities, m_done))) {
                }

                ~XMLInputFormat() {
                    try {
                        close();
                    } catch (...) {
                        // ignore any exceptions at this point because destructor should not throw
                    }
                }

                virtual osmium::io::Header header() override final {
                    osmium::thread::check_for_exception(m_parser_future);
                    return m_header_promise.get_future().get();
                }

                osmium::memory::Buffer read() override {
                    osmium::memory::Buffer buffer;
                    if (!m_done || !m_queue.empty()) {
                        m_queue.wait_and_pop(buffer);
                    }

                    osmium::thread::check_for_exception(m_parser_future);
                    return buffer;
                }

                void close() override {
                    m_done = true;
                    osmium::thread::wait_until_done(m_parser_future);
                }

            }; // class XMLInputFormat

            namespace {

// we want the register_input_format() function to run, setting the variable
// is only a side-effect, it will never be used
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
                const bool registered_xml_input = osmium::io::detail::InputFormatFactory::instance().register_input_format(osmium::io::file_format::xml,
                    [](const osmium::io::File& file, osmium::osm_entity_bits::type read_which_entities, osmium::thread::Queue<std::string>& input_queue) {
                        return new osmium::io::detail::XMLInputFormat(file, read_which_entities, input_queue);
                });
#pragma GCC diagnostic pop

            } // anonymous namespace

        } // namespace detail

    } // namespace io

} // namespace osmium

#endif // OSMIUM_IO_DETAIL_XML_INPUT_FORMAT_HPP
